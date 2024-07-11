# Pause/Resume functionality with position capture/restore
#
# Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class PauseResume:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.recover_velocity = config.getfloat('recover_velocity', 50.)
        self.v_sd = None
        self.is_paused = False
        self.sd_paused = False
        self.pause_command_sent = False
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.gcode.register_command("PAUSE", self.cmd_PAUSE)
        self.gcode.register_command("RESUME", self.cmd_RESUME)
        self.gcode.register_command("CLEAR_PAUSE", self.cmd_CLEAR_PAUSE)
        self.gcode.register_command("CANCEL_PRINT", self.cmd_CANCEL_PRINT)
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint("pause_resume/cancel",
                                   self._handle_cancel_request)
        webhooks.register_endpoint("pause_resume/pause",
                                   self._handle_pause_request)
        webhooks.register_endpoint("pause_resume/resume",
                                   self._handle_resume_request)
    def handle_ready(self):
        self.v_sd = self.printer.lookup_object('virtual_sdcard', None)
    def _handle_cancel_request(self, web_request):
        self.gcode.run_script("CANCEL_PRINT")
    def _handle_pause_request(self, web_request):
        self.gcode.run_script("PAUSE")
    def _handle_resume_request(self, web_request):
        self.gcode.run_script("RESUME")
    def get_status(self, eventtime):
        return {
            'is_paused': self.is_paused
        }
    def send_pause_command(self):
        # This sends the appropriate pause command from an event.  Note
        # the difference between pause_command_sent and is_paused, the
        # module isn't officially paused until the PAUSE gcode executes.
        if not self.pause_command_sent:
            if self.v_sd is not None and self.v_sd.is_active():
                # Printing from virtual sd, run pause command
                self.sd_paused = True
                self.v_sd.do_pause()
            else:
                self.sd_paused = False
                self.gcode.respond_info("action:paused")
            self.pause_command_sent = True
    def cmd_PAUSE(self, gcmd):
        if self.is_paused:
            gcmd.respond_info("Print already paused")
            return
        self.send_pause_command()
        self.gcode.run_script_from_command("SAVE_GCODE_STATE STATE=PAUSE_STATE")
        self.is_paused = True
    def cmd_RESUME(self, gcmd):
        if not self.is_paused:
            gcmd.respond_info("Print is not paused, resume aborted")
            return
        velocity = gcmd.get_float('VELOCITY', self.recover_velocity)
        self.gcode.run_script_from_command(
            "RESTORE_GCODE_STATE STATE=PAUSE_STATE MOVE=1 MOVE_SPEED=%.4f"
            % (velocity))
        self.is_paused = False
        self.pause_command_sent = False
        if self.sd_paused:
            # Printing from virtual sd, run pause command
            self.v_sd.cmd_M24(gcmd)
        else:
            gcmd.respond_info("action:resumed")
    def cmd_CLEAR_PAUSE(self, gcmd):
        self.is_paused = self.pause_command_sent = False
    def cmd_CANCEL_PRINT(self, gcmd):
        self.cmd_PAUSE(gcmd)
        if not self.sd_paused:
            gcmd.respond_info("action:cancel")
        self.cmd_CLEAR_PAUSE(gcmd)

def load_config(config):
    return PauseResume(config)
