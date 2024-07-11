#!/bin/bash
SRCDIR=klipper
PYTHONDIR=klippy-env

echo Running apt-get update
sudo apt-get update
echo Running apt-get install
sudo apt-get install --yes virtualenv python-dev libffi-dev build-essential libncurses-dev libusb-dev libusb-1.0-0
echo Creating python environment
[ ! -d ${PYTHONDIR} ] && virtualenv -p python2 ${PYTHONDIR}
echo Installing python dependencies
${PYTHONDIR}/bin/pip install -r ${SRCDIR}/scripts/klippy-requirements.txt

