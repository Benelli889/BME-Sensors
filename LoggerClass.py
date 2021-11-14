#!/usr/bin/python
import logging, datetime, os, sys
from TimerClass import TimerClass
from datetime import timedelta

class LoggerClass():
    Timer = TimerClass()

    def __init__(self):
        self.Timer.SetTimerDuration(timedelta(weeks=0, days=0, seconds=0, microseconds=0, milliseconds=0, minutes=10, hours=0))

    def Configure(self, path_to_log_directory, FileName):

        # Logger Text Format
        log_filename = datetime.datetime.now().strftime('%Y-%m-%d') + " -  " + FileName + '.log'
        importer_logger = logging.getLogger('importer_logger')
        importer_logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s : %(levelname)s : %(message)s')

        #File Handler
        fh = logging.FileHandler(filename=os.path.join(path_to_log_directory, log_filename))
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(formatter)
        importer_logger.addHandler(fh)

        # Stream Handler
        sh = logging.StreamHandler(sys.stdout)
        sh.setLevel(logging.ERROR)
        sh.setFormatter(formatter)
        importer_logger.addHandler(sh)

    def Msg(self, Msg):
        LogHandel = logging.getLogger('importer_logger')
        LogHandel.debug(Msg)

    def MsgFrequency(self, Msg):
        LogHandel = logging.getLogger('importer_logger')

        if self.Timer.State() == self.Timer.STOPPED:
            self.Timer.Start()

        if self.Timer.State() == self.Timer.STARTED:
            if self.Timer.TimerRunUp():
                LogHandel.debug(Msg)