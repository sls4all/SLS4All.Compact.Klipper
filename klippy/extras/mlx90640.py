# Support for I2C MLX90640 temperature sensor/camera
#
# Copyright (C) 2021  Pavel Dyntera
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import copy
from struct import unpack
import math
from . import bus

MLX90640_CHIP_ADDR = 0x33
MLX90640_I2C_SPEED = 1000000
MLX90640_REPORT_TIME = .05
MLX90640_SCALEALPHA = 0.000001
MLX90640_BLOCK_SIZE = 16
MAX_REASONABLE_TEMP = 1000

class error(Exception):
    pass

class paramsMLX90640:
    def __init__(self, eeData):
        self.ksTo = [0.0] * 5
        self.ct = [0] * 5
        self.alpha = [0] * 768    
        self.offset = [0] * 768    
        self.kta = [0] * 768
        self.kv = [0] * 768
        self.cpAlpha = [0.0] * 2
        self.cpOffset = [0] * 2
        self.ilChessC = [0.0] * 3 
        self.brokenPixels = [0] * 5
        self.outlierPixels = [0] * 5  

        self.ExtractVDDParameters(eeData)
        self.ExtractPTATParameters(eeData)
        self.ExtractGainParameters(eeData)
        self.ExtractTgcParameters(eeData)
        self.ExtractResolutionParameters(eeData)
        self.ExtractKsTaParameters(eeData)
        self.ExtractKsToParameters(eeData)
        self.ExtractCPParameters(eeData)
        self.ExtractAlphaParameters(eeData)
        self.ExtractOffsetParameters(eeData)
        self.ExtractKtaPixelParameters(eeData)
        self.ExtractKvPixelParameters(eeData)
        self.ExtractCILCParameters(eeData)
        self.ExtractDeviatingPixels(eeData)  

    def ExtractVDDParameters(self, eeData):
        kVdd = eeData[51]
        kVdd = (eeData[51] & 0xFF00) >> 8
        if kVdd > 127:
            kVdd = kVdd - 256
        kVdd = 32 * kVdd
        vdd25 = eeData[51] & 0x00FF
        vdd25 = ((vdd25 - 256) << 5) - 8192
        
        self.kVdd = kVdd
        self.vdd25 = vdd25 

    def ExtractPTATParameters(self, eeData):
        KvPTAT = float((eeData[50] & 0xFC00) >> 10)
        if KvPTAT > 31:
            KvPTAT = KvPTAT - 64
        KvPTAT = KvPTAT/4096
        
        KtPTAT = eeData[50] & 0x03FF
        if KtPTAT > 511:
            KtPTAT = KtPTAT - 1024
        KtPTAT = KtPTAT/8
        vPTAT25 = eeData[49]
        alphaPTAT = (eeData[16] & 0xF000) / pow(2.0, 14) + 8.0
        self.KvPTAT = KvPTAT
        self.KtPTAT = KtPTAT    
        self.vPTAT25 = vPTAT25
        self.alphaPTAT = alphaPTAT

    def ExtractGainParameters(self, eeData):
        gainEE = eeData[48]
        if(gainEE > 32767):
            gainEE = gainEE -65536
        self.gainEE = gainEE    

    def ExtractTgcParameters(self, eeData):
        tgc = eeData[60] & 0x00FF
        if(tgc > 127):
            tgc = tgc - 256
        tgc = tgc / 32.0
        
        self.tgc = tgc        

    def ExtractResolutionParameters(self, eeData):
        resolutionEE = (eeData[56] & 0x3000) >> 12    
        self.resolutionEE = resolutionEE

    def ExtractKsTaParameters(self, eeData):
        KsTa = (eeData[60] & 0xFF00) >> 8
        if(KsTa > 127):
            KsTa = KsTa -256
        KsTa = KsTa / 8192.0
        self.KsTa = KsTa

    def ExtractKsToParameters(self, eeData):
        step = ((eeData[63] & 0x3000) >> 12) * 10
        
        self.ct = [0] * 5
        self.ct[0] = -40
        self.ct[1] = 0
        self.ct[2] = (eeData[63] & 0x00F0) >> 4
        self.ct[3] = (eeData[63] & 0x0F00) >> 8    
        
        self.ct[2] = self.ct[2]*step
        self.ct[3] = self.ct[2] + self.ct[3]*step
        self.ct[4] = 400
        
        KsToScale = (eeData[63] & 0x000F) + 8
        KsToScale = 1 << KsToScale
        
        self.ksTo = [0.0] * 5
        self.ksTo[0] = float(eeData[61] & 0x00FF)
        self.ksTo[1] = float((eeData[61] & 0xFF00) >> 8)
        self.ksTo[2] = float(eeData[62] & 0x00FF)
        self.ksTo[3] = float((eeData[62] & 0xFF00) >> 8)      
        
        for i in range(0, 4):
            if(self.ksTo[i] > 127):
                self.ksTo[i] = self.ksTo[i] - 256
            self.ksTo[i] = self.ksTo[i] / KsToScale
        
        self.ksTo[4] = -0.0002

    def ExtractAlphaParameters(self, eeData):
        accRow = [0] * 24
        accColumn = [0] * 32
        p = 0
        alphaTemp = [0.0] * 768

        accRemScale = eeData[32] & 0x000F
        accColumnScale = (eeData[32] & 0x00F0) >> 4
        accRowScale = (eeData[32] & 0x0F00) >> 8
        alphaScale = ((eeData[32] & 0xF000) >> 12) + 30
        alphaRef = eeData[33]
        
        for i in range(0, 6):
            p = i * 4
            accRow[p + 0] = (eeData[34 + i] & 0x000F)
            accRow[p + 1] = (eeData[34 + i] & 0x00F0) >> 4
            accRow[p + 2] = (eeData[34 + i] & 0x0F00) >> 8
            accRow[p + 3] = (eeData[34 + i] & 0xF000) >> 12
        
        for i in range(0, 24):
            if (accRow[i] > 7):
                accRow[i] = accRow[i] - 16
        
        for i in range(0, 8):
            p = i * 4
            accColumn[p + 0] = (eeData[40 + i] & 0x000F)
            accColumn[p + 1] = (eeData[40 + i] & 0x00F0) >> 4
            accColumn[p + 2] = (eeData[40 + i] & 0x0F00) >> 8
            accColumn[p + 3] = (eeData[40 + i] & 0xF000) >> 12
        
        for i in range(0, 32):
            if (accColumn[i] > 7):
                accColumn[i] = accColumn[i] - 16

        for i in range(0, 24):
            for j in range(0, 32):
                p = 32 * i +j
                alphaTemp[p] = float((eeData[64 + p] & 0x03F0) >> 4)
                if (alphaTemp[p] > 31):
                    alphaTemp[p] = alphaTemp[p] - 64
                alphaTemp[p] = alphaTemp[p]*(1 << accRemScale)
                alphaTemp[p] = (alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) + alphaTemp[p])
                alphaTemp[p] = alphaTemp[p] / pow(2.0,alphaScale)
                alphaTemp[p] = alphaTemp[p] - self.tgc * (self.cpAlpha[0] + self.cpAlpha[1])/2
                alphaTemp[p] = MLX90640_SCALEALPHA/alphaTemp[p]
        
        temp = alphaTemp[0]
        for i in range(1, 768):
            if (alphaTemp[i] > temp):
                temp = alphaTemp[i]
        
        alphaScale = 0
        while(temp < 32767.4):
            temp = temp*2
            alphaScale = alphaScale + 1
        
        for i in range(0, 768):
            temp = alphaTemp[i] * pow(2.0,alphaScale)        
            self.alpha[i] = (temp + 0.5)        
        
        self.alphaScale = alphaScale      

    def ExtractOffsetParameters(self, eeData):
        occRow = [0] * 24
        occColumn = [0] * 32
        p = 0

        occRemScale = (eeData[16] & 0x000F)
        occColumnScale = (eeData[16] & 0x00F0) >> 4
        occRowScale = (eeData[16] & 0x0F00) >> 8
        offsetRef = eeData[17]
        if (offsetRef > 32767):
            offsetRef = offsetRef - 65536
        
        for i in range(0, 6):
            p = i * 4
            occRow[p + 0] = (eeData[18 + i] & 0x000F)
            occRow[p + 1] = (eeData[18 + i] & 0x00F0) >> 4
            occRow[p + 2] = (eeData[18 + i] & 0x0F00) >> 8
            occRow[p + 3] = (eeData[18 + i] & 0xF000) >> 12
        
        for i in range(0, 24):
            if (occRow[i] > 7):
                occRow[i] = occRow[i] - 16
        
        for i in range(0, 8):
            p = i * 4
            occColumn[p + 0] = (eeData[24 + i] & 0x000F)
            occColumn[p + 1] = (eeData[24 + i] & 0x00F0) >> 4
            occColumn[p + 2] = (eeData[24 + i] & 0x0F00) >> 8
            occColumn[p + 3] = (eeData[24 + i] & 0xF000) >> 12
        
        for i in range(0, 32):
            if (occColumn[i] > 7):
                occColumn[i] = occColumn[i] - 16

        for i in range(0, 24):
            for j in range(0, 32):
                p = 32 * i +j
                self.offset[p] = (eeData[64 + p] & 0xFC00) >> 10
                if (self.offset[p] > 31):
                    self.offset[p] = self.offset[p] - 64
                self.offset[p] = self.offset[p]*(1 << occRemScale)
                self.offset[p] = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + self.offset[p])

    def ExtractKtaPixelParameters(self, eeData):
        p = 0
        KtaRC = [0] * 4
        ktaTemp = [0.0] * 768
        
        KtaRoCo = (eeData[54] & 0xFF00) >> 8
        if (KtaRoCo > 127):
            KtaRoCo = KtaRoCo - 256
        KtaRC[0] = KtaRoCo
        
        KtaReCo = (eeData[54] & 0x00FF)
        if (KtaReCo > 127):
            KtaReCo = KtaReCo - 256
        KtaRC[2] = KtaReCo
        
        KtaRoCe = (eeData[55] & 0xFF00) >> 8
        if (KtaRoCe > 127):
            KtaRoCe = KtaRoCe - 256
        KtaRC[1] = KtaRoCe
        
        KtaReCe = (eeData[55] & 0x00FF)
        if (KtaReCe > 127):
            KtaReCe = KtaReCe - 256
        KtaRC[3] = KtaReCe
    
        ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8
        ktaScale2 = (eeData[56] & 0x000F)

        for i in range(0, 24):
            for j in range(0, 32):
                p = 32 * i +j
                split = 2*(p/32 - (p/64)*2) + p%2
                ktaTemp[p] = (eeData[64 + p] & 0x000E) >> 1
                if (ktaTemp[p] > 3):
                    ktaTemp[p] = ktaTemp[p] - 8
                ktaTemp[p] = ktaTemp[p] * (1 << ktaScale2)
                ktaTemp[p] = KtaRC[split] + ktaTemp[p]
                ktaTemp[p] = ktaTemp[p] / pow(2.0, ktaScale1)
                #ktaTemp[p] = ktaTemp[p] * mlx90640->offset[p]
        
        temp = abs(ktaTemp[0])
        for i in range(1, 768):
            if (abs(ktaTemp[i]) > temp):
                temp = abs(ktaTemp[i])
        
        ktaScale1 = 0
        while(temp < 63.4):
            temp = temp*2
            ktaScale1 = ktaScale1 + 1
        
        for i in range(0, 768):
            temp = ktaTemp[i] * pow(2.0, ktaScale1)
            if (temp < 0):
                self.kta[i] = (temp - 0.5)
            else:
                self.kta[i] = (temp + 0.5)
        
        self.ktaScale = ktaScale1           

    def ExtractKvPixelParameters(self, eeData):
        p = 0
        KvT = [0] * 4
        kvTemp = [0] * 768

        KvRoCo = (eeData[52] & 0xF000) >> 12
        if (KvRoCo > 7):
            KvRoCo = KvRoCo - 16
        KvT[0] = KvRoCo
        
        KvReCo = (eeData[52] & 0x0F00) >> 8
        if (KvReCo > 7):
            KvReCo = KvReCo - 16
        KvT[2] = KvReCo
        
        KvRoCe = (eeData[52] & 0x00F0) >> 4
        if (KvRoCe > 7):
            KvRoCe = KvRoCe - 16
        KvT[1] = KvRoCe
        
        KvReCe = (eeData[52] & 0x000F)
        if (KvReCe > 7):
            KvReCe = KvReCe - 16
        KvT[3] = KvReCe
    
        kvScale = (eeData[56] & 0x0F00) >> 8

        for i in range(0, 24):
            for j in range(0, 32):
                p = 32 * i +j
                split = 2*(p/32 - (p/64)*2) + p%2
                kvTemp[p] = KvT[split]
                kvTemp[p] = kvTemp[p] / pow(2.0,kvScale)
                #kvTemp[p] = kvTemp[p] * mlx90640->offset[p]
        
        temp = abs(kvTemp[0])
        for i in range(1, 768):
            if (abs(kvTemp[i]) > temp):
                temp = abs(kvTemp[i])
        
        kvScale = 0
        while(temp < 63.4):
            temp = temp*2
            kvScale = kvScale + 1
        
        for i in range(0, 768):
            temp = kvTemp[i] * pow(2.0, kvScale)
            if (temp < 0):
                self.kv[i] = (temp - 0.5)
            else:
                self.kv[i] = (temp + 0.5)
        
        self.kvScale = kvScale        

    def ExtractCPParameters(self, eeData):
        alphaSP = [0.0] * 2
        offsetSP = [0] * 2

        alphaScale = ((eeData[32] & 0xF000) >> 12) + 27
        
        offsetSP[0] = (eeData[58] & 0x03FF)
        if (offsetSP[0] > 511):
            offsetSP[0] = offsetSP[0] - 1024
        
        offsetSP[1] = (eeData[58] & 0xFC00) >> 10
        if (offsetSP[1] > 31):
            offsetSP[1] = offsetSP[1] - 64
        offsetSP[1] = offsetSP[1] + offsetSP[0] 
        
        alphaSP[0] = float(eeData[57] & 0x03FF)
        if (alphaSP[0] > 511):
            alphaSP[0] = alphaSP[0] - 1024
        alphaSP[0] = alphaSP[0] /  pow(2.0,alphaScale)
        
        alphaSP[1] = float((eeData[57] & 0xFC00) >> 10)
        if (alphaSP[1] > 31):
            alphaSP[1] = alphaSP[1] - 64
        alphaSP[1] = (1 + alphaSP[1]/128) * alphaSP[0]
        
        cpKta = (eeData[59] & 0x00FF)
        if (cpKta > 127):
            cpKta = cpKta - 256
        ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8    
        self.cpKta = cpKta / pow(2.0,ktaScale1)
        
        cpKv = (eeData[59] & 0xFF00) >> 8
        if (cpKv > 127):
            cpKv = cpKv - 256
        kvScale = (eeData[56] & 0x0F00) >> 8
        self.cpKv = cpKv / pow(2.0,kvScale)
        
        self.cpAlpha[0] = alphaSP[0]
        self.cpAlpha[1] = alphaSP[1]
        self.cpOffset[0] = offsetSP[0]
        self.cpOffset[1] = offsetSP[1]  

    def ExtractCILCParameters(self, eeData):
        ilChessC = [0.0] * 3
        
        calibrationModeEE = (eeData[10] & 0x0800) >> 4
        calibrationModeEE = calibrationModeEE ^ 0x80

        ilChessC[0] = float(eeData[53] & 0x003F)
        if (ilChessC[0] > 31):
            ilChessC[0] = ilChessC[0] - 64
        ilChessC[0] = ilChessC[0] / 16
        
        ilChessC[1] = float((eeData[53] & 0x07C0) >> 6)
        if (ilChessC[1] > 15):
            ilChessC[1] = ilChessC[1] - 32
        ilChessC[1] = ilChessC[1] / 2
        
        ilChessC[2] = float((eeData[53] & 0xF800) >> 11)
        if (ilChessC[2] > 15):
            ilChessC[2] = ilChessC[2] - 32
        ilChessC[2] = ilChessC[2] / 8
        
        self.calibrationModeEE = calibrationModeEE
        self.ilChessC[0] = ilChessC[0]
        self.ilChessC[1] = ilChessC[1]
        self.ilChessC[2] = ilChessC[2]

    def ExtractDeviatingPixels(self, eeData):
        pixCnt = 0
        brokenPixCnt = 0
        outlierPixCnt = 0
        warn = 0

        for pixCnt in range(0, 5):
            self.brokenPixels[pixCnt] = 0xFFFF
            self.outlierPixels[pixCnt] = 0xFFFF
            
        pixCnt = 0    
        while (pixCnt < 768 and brokenPixCnt < 5 and outlierPixCnt < 5):
            if(eeData[pixCnt+64] == 0):
                self.brokenPixels[brokenPixCnt] = pixCnt
                brokenPixCnt = brokenPixCnt + 1
            elif((eeData[pixCnt+64] & 0x0001) != 0):
                self.outlierPixels[outlierPixCnt] = pixCnt
                outlierPixCnt = outlierPixCnt + 1
            pixCnt = pixCnt + 1
        
        if(brokenPixCnt > 4):  
            warn = -3
        elif(outlierPixCnt > 4):  
            warn = -4
        elif((brokenPixCnt + outlierPixCnt) > 4):  
            warn = -5
        else:
            for pixCnt in range(0, brokenPixCnt):
                for i in range(pixCnt+1, i<brokenPixCnt):
                    warn = self.CheckAdjacentPixels(self.brokenPixels[pixCnt],self.brokenPixels[i])
                    if(warn != 0):
                        return warn
            
            for pixCnt in range(0, outlierPixCnt):
                for i in range(pixCnt+1, i<brokenPixCnt):
                    warn = self.CheckAdjacentPixels(self.outlierPixels[pixCnt],self.outlierPixels[i])
                    if(warn != 0):
                        return warn
            
            for pixCnt in range(0, outlierPixCnt):
                for i in range(pixCnt+1, i<brokenPixCnt):
                    warn = self.CheckAdjacentPixels(self.brokenPixels[pixCnt],self.outlierPixels[i])
                    if(warn != 0):
                        return warn
        return warn

    def GetMedian(self, values,  n):
        for i in range(0, n-1):
            for j in range(i+1, n):
                if(values[j] < values[i]):
                    temp = values[i]
                    values[i] = values[j]
                    values[j] = temp
        if(n%2==0):
            return ((values[n/2] + values[n/2 - 1]) / 2.0)
        else:
            return values[n/2]

    def IsPixelBad(self, pixel, params):
        for i in range(0, 5):
            if(pixel == params.outlierPixels[i] or pixel == params.brokenPixels[i]):
                return 1
        return 0;     

    def CheckAdjacentPixels(self, pix1, pix2):
        pixPosDif = pix1 - pix2
        if(pixPosDif > -34 and pixPosDif < -30):
            return -6
        if(pixPosDif > -2 and pixPosDif < 2):
            return -6
        if(pixPosDif > 30 and pixPosDif < 34):
            return -6
        return 0    

    def MLX90640_GetTa(self, frameData):
        vdd = self.MLX90640_GetVdd(frameData)
        ptat = float(frameData[800])
        if ptat > 32767:
            ptat = ptat - 65536
    
        ptatArt = float(frameData[768])
        if ptatArt > 32767:
            ptatArt = ptatArt - 65536
        ptatArt = (ptat / (ptat * self.alphaPTAT + ptatArt)) * pow(2.0, 18)
    
        ta = (ptatArt / (1 + self.KvPTAT * (vdd - 3.3)) - self.vPTAT25)
        ta = ta / float(self.KtPTAT) + 25
    
        return ta
    
    def MLX90640_GetVdd(self, frameData):
        vdd = frameData[810]
        if vdd > 32767:
            vdd = vdd - 65536
        resolutionRAM = (frameData[832] & 0x0C00) >> 10
        resolutionCorrection = pow(2.0, self.resolutionEE) / pow(2.0, resolutionRAM)
        vdd = (resolutionCorrection * vdd - self.vdd25) / float(self.kVdd) + 3.3
        return vdd

class mlx90640:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(config, MLX90640_CHIP_ADDR,
                                           MLX90640_I2C_SPEED)
        self.mcu = self.i2c.get_mcu()
        self.report_time = config.getfloat('mlx90640_report_time', MLX90640_REPORT_TIME)
        self.temp = self.min_temp = self.max_temp = 0.0
        self.temp_matrix = [0.0] * 768
        self.temp_emissivity = 1.0
        push_mode = config.getboolean("push_mode", default = False)
        self.printer.add_object("mlx90640 " + self.name, self)
        self._callbacks = []
        if not push_mode:
            self.sample_timer = self.reactor.register_timer(self._sample_mlx90640)
            self.printer.register_event_handler("klippy:connect",
                                                self.handle_connect)

        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("SET_MLX_MATRIX", self.cmd_set_matrix, no_extended_params=True)
        gcode.register_command("GET_MLX_MATRIX", self.cmd_get_matrix)
        wh = self.printer.lookup_object('webhooks')
        wh.register_endpoint("gcode/set_mlx_matrix", self.cmd_set_matrix_webhooks)

    def cmd_set_matrix_inner(self, values):
        max_temp = max(values)
        if max_temp > MAX_REASONABLE_TEMP:
            logging.exception("mlx90640: Externally set unreasonably high max temperature value, should try again")
            return
        self.temp_matrix = values
        self.temp = max_temp
        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "MLX90640 temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp))
        measured_time = self.reactor.monotonic()
        estimated_print_time = self.mcu.estimated_print_time(measured_time)
        for callback in self._callbacks:
            callback(estimated_print_time, self.temp)

    def cmd_set_matrix(self, gcmd):
        msg = gcmd.get_commandline()
        strValues = msg[14:].strip()
        values = map(float, strValues.split())
        if len(values) != 768:
            raise error("invalid matrix length")
        self.cmd_set_matrix_inner(values)

    def cmd_set_matrix_webhooks(self, web_request):
        strValues = web_request.get('VALUES')
        values = map(float, strValues.split())
        if len(values) != 768:
            web_request.set_error("invalid matrix length")
            return
        self.cmd_set_matrix_inner(values)

    def cmd_get_matrix(self, gcmd):
        res = "32 24"
        for v in self.temp_matrix:
            res += " %.2f" % v
        gcmd.respond_raw(res)

    def handle_connect(self):
        self._init_mlx90640()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callbacks.append(cb)

    def get_report_time_delta(self):
        return self.report_time

    def _init_mlx90640(self):
        self.MLX90640_SetSubPages()
        self.MLX90640_SetChessMode()
        self.MLX90640_SetRefreshRate(0b011) # 8fps
        dump = self.MLX90640_DumpEE()
        self.params = paramsMLX90640(dump)

    def _sample_mlx90640(self, eventtime):
        isError = True
        try:
            frame = self.MLX90640_GetFrameData(True)
            if frame is not None:
                eTa = self.params.MLX90640_GetTa(frame)
                temp_matrix = copy.deepcopy(self.temp_matrix)
                self.MLX90640_CalculateTo(frame, self.params, self.temp_emissivity, eTa, temp_matrix)
                self.MLX90640_BadPixelsCorrection(self.params.brokenPixels, temp_matrix, 1, self.params)
                self.MLX90640_BadPixelsCorrection(self.params.outlierPixels, temp_matrix, 1, self.params)
                max_temp = max(temp_matrix)
                if max_temp > MAX_REASONABLE_TEMP:
                    logging.exception("mlx90640: Read unreasonably high max temperature value, will try again")
                else:
                    isError = False
                    self.temp_matrix = temp_matrix
                    self.temp = max_temp
        except Exception:
            isError = True
            logging.exception("mlx90640: Error reading data, will try again")

        measured_time = self.reactor.monotonic()
        if not isError:
            if self.temp < self.min_temp or self.temp > self.max_temp:
                self.printer.invoke_shutdown(
                    "MLX90640 temperature %0.1f outside range of %0.1f:%.01f"
                    % (self.temp, self.min_temp, self.max_temp))
            estimated_print_time = self.mcu.estimated_print_time(measured_time)
            for callback in self._callbacks:
                callback(estimated_print_time, self.temp)
    
        return measured_time + self.report_time

    def MLX90640_TriggerMeasurement(self):
        ctrlReg = self.MLX90640_I2CRead(0x800D, 1, may_fail=True)
        if ctrlReg is None:
            return
        ctrlReg[0] |= 0x8000
        res = self.MLX90640_I2CWrite(0x800D, ctrlReg, may_fail=True)
        if not res:
            return
        self.MLX90640_I2CWrite(0x06, 0x00, may_fail=True) # general reset

    def MLX90640_CalculateTo(self, frameData, params, emissivity, tr, result):
        irDataCP = [0.0] * 2
        alphaCorrR = [0.0] * 4
        
        subPage = frameData[833]
        vdd = params.MLX90640_GetVdd(frameData)
        ta = params.MLX90640_GetTa(frameData)

        ta4 = (ta + 273.15)
        ta4 = ta4 * ta4
        ta4 = ta4 * ta4
        tr4 = (tr + 273.15)
        tr4 = tr4 * tr4
        tr4 = tr4 * tr4
        taTr = tr4 - (tr4-ta4)/float(emissivity)
        
        ktaScale = pow(2.0,params.ktaScale)
        kvScale = pow(2.0,params.kvScale)
        alphaScale = pow(2.0,params.alphaScale)
        
        alphaCorrR[0] = 1.0 / (1 + params.ksTo[0] * 40)
        alphaCorrR[1] = 1.0
        alphaCorrR[2] = (1.0 + params.ksTo[1] * params.ct[2])
        alphaCorrR[3] = alphaCorrR[2] * (1.0 + params.ksTo[2] * (params.ct[3] - params.ct[2]))
        
        #//------------------------- Gain calculation -----------------------------------    
        gain = frameData[778]
        if(gain > 32767):
            gain = gain - 65536
        
        gain = float(params.gainEE) / gain; 
    
        #//------------------------- To calculation -------------------------------------    
        mode = (frameData[832] & 0x1000) >> 5
        
        irDataCP[0] = frameData[776];  
        irDataCP[1] = frameData[808]
        for i in range(0, 2):
            if(irDataCP[i] > 32767):
                irDataCP[i] = irDataCP[i] - 65536
            irDataCP[i] = irDataCP[i] * gain
        irDataCP[0] = irDataCP[0] - params.cpOffset[0] * (1 + params.cpKta * (ta - 25)) * (1 + params.cpKv * (vdd - 3.3))
        if( mode ==  params.calibrationModeEE):
            irDataCP[1] = irDataCP[1] - params.cpOffset[1] * (1 + params.cpKta * (ta - 25)) * (1 + params.cpKv * (vdd - 3.3))
        else:
            irDataCP[1] = irDataCP[1] - (params.cpOffset[1] + params.ilChessC[0]) * (1 + params.cpKta * (ta - 25)) * (1 + params.cpKv * (vdd - 3.3))

        for pixelNumber in range(0, 768):
            ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2
            chessPattern = ilPattern ^ (pixelNumber - (pixelNumber/2)*2) 
            conversionPattern = ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) * (1 - 2 * ilPattern)
            
            if(mode == 0):
                pattern = ilPattern; 
            else:
                pattern = chessPattern; 
            
            if(pattern == subPage):
                irData = frameData[pixelNumber]
                if(irData > 32767):
                    irData = irData - 65536
                irData = irData * gain
                
                kta = params.kta[pixelNumber]/ktaScale
                kv = params.kv[pixelNumber]/kvScale
                irData = irData - params.offset[pixelNumber]*(1 + kta*(ta - 25))*(1 + kv*(vdd - 3.3))
                
                if(mode !=  params.calibrationModeEE):
                    irData = irData + params.ilChessC[2] * (2 * ilPattern - 1) - params.ilChessC[1] * conversionPattern 
        
                irData = irData - params.tgc * irDataCP[subPage]
                irData = irData / float(emissivity)
                
                alphaCompensated = MLX90640_SCALEALPHA*alphaScale/params.alpha[pixelNumber]
                alphaCompensated = alphaCompensated*(1 + params.KsTa * (ta - 25))
                            
                Sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr)
                Sx = math.sqrt(math.sqrt(Sx)) * params.ksTo[1]
                
                To = math.sqrt(math.sqrt(irData/(alphaCompensated * (1 - params.ksTo[1] * 273.15) + Sx) + taTr)) - 273.15                     
                        
                if(To < params.ct[1]):
                    rrange = 0
                elif(To < params.ct[2]):   
                    rrange = 1;            
                elif(To < params.ct[3]):
                    rrange = 2;            
                else:
                    rrange = 3;            
                
                To = math.sqrt(math.sqrt(irData / (alphaCompensated * alphaCorrR[rrange] * (1.0 + params.ksTo[rrange] * (To - params.ct[rrange]))) + taTr)) - 273.15
                            
                result[pixelNumber] = To
        return result

    def MLX90640_I2CRead(self, startAddress, nMemAddressRead, may_fail = False):
        cmd = [startAddress >> 8, startAddress & 0xFF]
        if not may_fail:
            params = self.i2c.i2c_read(cmd, nMemAddressRead * 2)
        else:
            params = self.i2c.i2c_read_ok(cmd, nMemAddressRead * 2)
            if not params["ok"]:
                return None
        data = bytearray(params['response'])
        res = [0] * nMemAddressRead
        for i in range(0, nMemAddressRead):
            p = (i << 1)
            res[i] = (data[p] << 8) + data[p+1]
        return res

    def MLX90640_I2CWrite(self, startAddress, data, may_fail = False):
        cmd = [startAddress >> 8, startAddress & 0xFF, data >> 8, data & 0xFF]
        if not may_fail:
            params = self.i2c.i2c_write(cmd)
        else:
            params = self.i2c.i2c_write_ok(cmd)
            if not params["ok"]:
                return False
        dataCheck = self.MLX90640_I2CRead(startAddress, 1, may_fail=may_fail)
        # if (dataCheck[0] != data):
        #     raise error("write data check failed")
        return True

    def MLX90640_SetChessMode(self):
        controlRegister1 = self.MLX90640_I2CRead(0x800D, 1)        
        value = (controlRegister1[0] | 0x1000)
        self.MLX90640_I2CWrite(0x800D, value)

    def MLX90640_DumpEE(self):
        #dump = self.MLX90640_I2CRead(0x2400, 832)
        dump = []
        for pos in range(0, 832, MLX90640_BLOCK_SIZE):
            dump += self.MLX90640_I2CRead(0x2400 + (pos<<0), MLX90640_BLOCK_SIZE)
        return dump

    # def MLX90640_SynchFrame(self):
    #     self.MLX90640_I2CWrite(0x8000, 0x0030)
    #     dataReady = 0
    #     while dataReady == 0:
    #         statusRegister = self.MLX90640_I2CRead(0x8000, 1)
    #         dataReady = statusRegister[0] & 0x0008

    def MLX90640_BadPixelsCorrection(self, pixels, to, mode, params):
        ap = [0.0] * 4
        pix = 0
        while(pixels[pix] != 0xFFFF):
            line = pixels[pix]>>5
            column = pixels[pix] - (line<<5)
            if(mode == 1):
                if(line == 0):
                    if(column == 0):
                        to[pixels[pix]] = to[33]
                    elif(column == 31):
                        to[pixels[pix]] = to[62]
                    else:
                        to[pixels[pix]] = (to[pixels[pix]+31] + to[pixels[pix]+33])/2.0
                elif(line == 23):
                    if(column == 0):
                        to[pixels[pix]] = to[705]
                    elif(column == 31):
                        to[pixels[pix]] = to[734]
                    else:
                        to[pixels[pix]] = (to[pixels[pix]-33] + to[pixels[pix]-31])/2.0
                if(column == 0):
                    to[pixels[pix]] = (to[pixels[pix]-31] + to[pixels[pix]+33])/2.0
                elif(column == 31):
                    to[pixels[pix]] = (to[pixels[pix]-33] + to[pixels[pix]+31])/2.0
                else:
                    ap[0] = to[pixels[pix]-33]
                    ap[1] = to[pixels[pix]-31]
                    ap[2] = to[pixels[pix]+31]
                    ap[3] = to[pixels[pix]+33]
                    to[pixels[pix]] = self.GetMedian(ap,4)
            else:
                if(column == 0):
                    to[pixels[pix]] = to[pixels[pix]+1]
                elif(column == 1 or column == 30):
                    to[pixels[pix]] = (to[pixels[pix]-1]+to[pixels[pix]+1])/2.0
                elif(column == 31):
                    to[pixels[pix]] = to[pixels[pix]-1]
                else:
                    if(self.IsPixelBad(pixels[pix]-2,params) == 0 and self.IsPixelBad(pixels[pix]+2,params) == 0):
                        ap[0] = to[pixels[pix]+1] - to[pixels[pix]+2]
                        ap[1] = to[pixels[pix]-1] - to[pixels[pix]-2]
                        if(abs(ap[0]) > abs(ap[1])):
                            to[pixels[pix]] = to[pixels[pix]-1] + ap[1]                        
                        else:
                            to[pixels[pix]] = to[pixels[pix]+1] + ap[0]
                    else:
                        to[pixels[pix]] = (to[pixels[pix]-1]+to[pixels[pix]+1])/2.0
            pix = pix + 1

    def MLX90640_GetFrameData(self, checkStatus=True):
        statusRegister = self.MLX90640_I2CRead(0x8000, 1, may_fail=True)
        if statusRegister is None:
            return None
        if (statusRegister[0] & 0x0008) == 0 and checkStatus:
            return None
        res = self.MLX90640_I2CWrite(0x8000, 0x0030, may_fail=True)
        if not res:
            return None
        
        # frameData = self.MLX90640_I2CRead(0x0400, 768)
        frameData = []
        for pos in range(0, 768, MLX90640_BLOCK_SIZE):
            partialData = self.MLX90640_I2CRead(0x0400 + (pos<<0), MLX90640_BLOCK_SIZE, may_fail=True)
            if partialData is None:
                return None
            frameData += partialData
        # data = self.MLX90640_I2CRead(0x0700, 64)
        data = []
        for pos in range(0, 64, MLX90640_BLOCK_SIZE):
            partialData = self.MLX90640_I2CRead(0x0700 + (pos<<0), MLX90640_BLOCK_SIZE, may_fail=True)
            if partialData is None:
                return None
            data += partialData
        controlRegister1 = self.MLX90640_I2CRead(0x800D, 1, may_fail=True)
        if controlRegister1 is None:
            return None
        if not self.ValidateAuxData(data):
            return None
        frameData += data
        frameData += [ controlRegister1[0], statusRegister[0] & 0x0001 ]
        if not self.ValidateFrameData(frameData):
            return None
        return frameData

    def MLX90640_SetRefreshRate(self, refreshRate):
        controlRegister1 = self.MLX90640_I2CRead(0x800D, 1)
        value = (controlRegister1[0] & 0b1111110001111111) | ((refreshRate & 0b111)<<7)
        self.MLX90640_I2CWrite(0x800D, value)

    def MLX90640_SetSubPages(self):
        controlRegister1 = self.MLX90640_I2CRead(0x800D, 1)
        value = (controlRegister1[0] & 0b11110110) | 0b0001
        self.MLX90640_I2CWrite(0x800D, value)

    def ValidateFrameData(self, frameData):
        line = 0
        for i in range(0, 768, 32):
            if (frameData[i] == 0x7FFF) and (line%2 == frameData[833]):
                return False
            line += 1
        return True

    def ValidateAuxData(self, auxData):
        if auxData[0] == 0x7FFF:
            return False 
    
        for i in range(8, 19):
            if auxData[i] == 0x7FFF:
                return False
    
        for i in range(20, 23):
            if auxData[i] == 0x7FFF:
                return False
    
        for i in range(24, 33):
            if auxData[i] == 0x7FFF:
                return False
    
        for i in range(40, 51):
            if auxData[i] == 0x7FFF:
                return False
    
        for i in range(52, 55):
            if auxData[i] == 0x7FFF:
                return False
    
        for i in range(56, 64):
            if auxData[i] == 0x7FFF:
                return False

        return True

    def get_status(self, eventtime):
        return {
            'temperature': self.temp,
        }

class mlx90640_box:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.temp = self.min_temp = self.max_temp = 0.0
        self.printer.add_object("mlx90640_box " + self.name, self)
        self.parent_name = config.get('parent')
        self.parent = self.printer.lookup_object("mlx90640 " + self.parent_name)
        self.parent.setup_callback(self.temperature_callback)
        self.minx = config.getint('minx')
        self.miny = config.getint('miny')
        self.maxx = config.getint('maxx')
        self.maxy = config.getint('maxy')
        self.mode = config.get('mode', "max")

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.parent.get_report_time_delta()

    def temperature_callback(self, read_time, temp):
        temp_matrix = self.parent.temp_matrix
        values = []
        for y in range(self.miny, self.maxy + 1):
            for x in range(self.minx, self.maxx + 1):
                v = temp_matrix[y * 32 + x]
                values.append(v)
        if self.mode == "max":
            self.temp = max(values)
        elif self.mode == "min":
            self.temp = min(values)
        elif self.mode == "avg":
            self.temp = sum(values) / len(values)
        else:
            raise error("Invalid mode")

        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "MLX90640 temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp))
        self._callback(read_time, self.temp)

def load_config_prefix(config):
    return mlx90640(config)

def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("mlx90640", mlx90640)
    pheaters.add_sensor_factory("mlx90640_box", mlx90640_box)
