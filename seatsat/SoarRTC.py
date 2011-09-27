#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Soar general artificial intelligence component

Copyright (C) 2010
    Yosuke Matsusaka
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

import os
import sys
import time
import codecs
import locale
import traceback
import types
import threading
import optparse
from pprint import pformat
from BeautifulSoup import BeautifulSoup
from BeautifulSoup import Tag
import OpenRTM_aist
import RTC
from seatsat.XableRTC import *
from Python_sml_ClientInterface import *
from seatsat.__init__ import __version__
from seatsat import utils
try:
    import gettext
    _ = gettext.translation(domain='seatsat', localedir=os.path.dirname(__file__)+'/../share/locale').ugettext
except:
    _ = lambda s: s

__doc__ = _('Soar general artificial intelligence component.')

class SoarWrap(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._loop = True
        self._kernel = None
        self._agent = None
        self._lock = threading.Lock()
        # create soar kernel
        with self._lock:
            self._kernel = Kernel.CreateKernelInNewThread()
            if self._kernel.HadError():
                print self._kernel.GetLastErrorDescription()
            self._agent = self._kernel.CreateAgent('soarrtc')
            if self._kernel.HadError():
                print self._kernel.GetLastErrorDescription()

    def run(self):
        #self._agent.ExecuteCommandLine('waitsnc --enable')
        #self._kernel.RunAllAgentsForever()
        while self._loop:
            with self._lock:
                self._kernel.RunAllAgents(1)
            time.sleep(0.1)
    def stop(self):
        self._loop = False
        with self._lock:
            self._kernel.StopAllAgents()

    def terminate(self):
        with self._lock:
            self._kernel.DestroyAgent(self._agent)
            self._kernel.Shutdown()

SoarRTC_spec = ["implementation_id", "SoarRTC",
                "type_name",         "SoarRTC",
                "description",       __doc__.encode('UTF-8'),
                "version",           __version__,
                "vendor",            "AIST",
                "category",          "communication",
                "activity_type",     "DataFlowComponent",
                "max_instance",      "10",
                "language",          "Python",
                "lang_type",         "script",
                ""]

class SoarRTC(XableRTC):
    def __init__(self, manager):
        XableRTC.__init__(self, manager)
        self._soar = None
        self._basewme = {}
        self._datawme = {}
        self._timewme = {}
        self._dataidwme = {}
        self._dataid = 0

    def onInitialize(self):
        OpenRTM_aist.DataFlowComponentBase.onInitialize(self)
        XableRTC.onInitialize(self)
        self._logger = OpenRTM_aist.Manager.instance().getLogbuf(self._properties.getProperty("instance_name"))
        self._logger.RTC_INFO("SoarRTC version " + __version__)
        self._logger.RTC_INFO("Copyright (C) 2010 Yosuke Matsusaka")
        # create outport for command
        self._commanddata = RTC.TimedString(RTC.Time(0,0), "")
        self._commandport = OpenRTM_aist.OutPort("command", self._commanddata)
        self.registerOutPort("command", self._commandport)
        self._soar = SoarWrap()
        return RTC.RTC_OK
    
    def onActivated(self, ec_id):
        OpenRTM_aist.DataFlowComponentBase.onActivated(self, ec_id)
        self._soar.start()
        return RTC.RTC_OK

    def onDeactivated(self, ec_id):
        OpenRTM_aist.DataFlowComponentBase.onDeactivated(self, ec_id)
        self._soar.stop()
        return RTC.RTC_OK

    def onFinalize(self):
        OpenRTM_aist.DataFlowComponentBase.onFinalize(self)
        self._soar.stop()
        self._soar.terminate()
        return RTC.RTC_OK

    def docRecur(self, doc, wme):
        agent = self._soar._agent
        if not isinstance(doc, BeautifulSoup) and not isinstance(doc, Tag):
            return
        wme2 = agent.CreateIdWME(wme, str(doc.name))
        for c in doc.contents:
            self.docRecur(c, wme2)
        for a in doc.attrs:
            try:
                v = float(a[1])
                agent.CreateFloatWME(wme2, str(a[0]), v)
            except ValueError:
                agent.CreateStringWME(wme2, str(a[0]), a[1].encode('UTF-8'))

    def onData(self, info, data):
        try:
            self._logger.RTC_INFO('got input: ' + pformat(info) + ', ' + pformat(data))
            t = data.tm.sec + data.tm.nsec * 1e-9
            with self._soar._lock:
                agent = self._soar._agent
                portid = (info['component'], info['port'])
                if portid not in self._basewme:
                    self._logger.RTC_INFO('First input from this port > create basic structure on WME')
                    iid = agent.GetInputLink()
                    wme = agent.CreateIdWME(iid, 'data')
                    self._basewme[portid] = wme
                    ot = type(data.data)
                    if ot in types.StringTypes:
                        if data.data[:5] == '<?xml':
                            self._logger.RTC_INFO('Parsing XML type input')
                            doc = BeautifulSoup(data.data)
                            wme2 = agent.CreateIdWME(wme, 'data')
                            self.docRecur(doc.first(), wme2)
                            self._datawme[portid] = wme2
                        else:
                            self._datawme[portid] = agent.CreateStringWME(wme, 'data', data.data)
                    elif ot == types.IntType:
                        self._datawme[portid] = agent.CreateIntWME(wme, 'data', data.data)
                    elif ot == types.FloatType:
                        self._datawme[portid] = agent.CreateFloatWME(wme, 'data', data.data)
                    else:
                        self._logger.RTC_ERROR('unsupported data type: ' + str(ot))
                    self._timewme[portid] = agent.CreateFloatWME(wme, 'time', t)
                    self._dataidwme[portid] = agent.CreateIntWME(wme, 'id', self._dataid)
                    for k, v in info.iteritems():
                        agent.CreateStringWME(wme, k, v)
                else:
                    agent.Update(self._timewme[portid], t)
                    agent.Update(self._dataidwme[portid], self._dataid)
                    if type(data.data) in types.StringTypes and data.data[:5] == '<?xml':
                        self._logger.RTC_INFO('Parsing XML type input')
                        doc = BeautifulSoup(data.data)
                        agent.DestroyWME(self._datawme[portid])
                        wme2 = agent.CreateIdWME(self._basewme[portid], 'data')
                        self.docRecur(doc.first(), wme2)
                        self._datawme[portid] = wme2
                    else:
                        agent.Update(self._datawme[portid], data.data)
                agent.Commit()
                self._dataid += 1
        except:
            self._logger.RTC_ERROR(traceback.format_exc())

    def onExecute(self, ec_id):
        OpenRTM_aist.DataFlowComponentBase.onExecute(self, ec_id)
        numberCommands = self._soar._agent.GetNumberCommands()
        for i in range(0, numberCommands):
            command = self._soar._agent.GetCommand(i)
            name  = command.GetCommandName()
            if name == "rtcout":
                port = command.GetParameterValue("port")
                self._commanddata.data = command.GetParameterValue("data")
                self._commandport.write()
            command.AddStatusComplete()
        self._soar._agent.ClearOutputLinkChanges()
        if self._soar._kernel.HadError():
            self._logger.RTC_ERROR(self._soar._kernel.GetLastErrorDescription())
        if self._soar._agent.HadError():
            self._logger.RTC_ERROR(self._soar._agent.GetLastErrorDescription())
        return RTC.RTC_OK

class SoarRTCManager:
    def __init__(self):
        encoding = locale.getpreferredencoding()
        sys.stdout = codecs.getwriter(encoding)(sys.stdout, errors = "replace")
        sys.stderr = codecs.getwriter(encoding)(sys.stderr, errors = "replace")

        parser = utils.MyParser(version=__version__, description=__doc__)
        utils.addmanageropts(parser)
        try:
            opts, args = parser.parse_args()
        except optparse.OptionError, e:
            print >>sys.stderr, 'OptionError:', e
            sys.exit(1)

        self._comp = None
        self._manager = OpenRTM_aist.Manager.init(utils.genmanagerargs(opts))
        self._manager.setModuleInitProc(self.moduleInit)
        self._manager.activateManager()

    def start(self):
        self._manager.runManager(False)

    def moduleInit(self, manager):
        profile=OpenRTM_aist.Properties(defaults_str=SoarRTC_spec)
        manager.registerFactory(profile, SoarRTC, OpenRTM_aist.Delete)
        self._comp = manager.createComponent("SoarRTC?exec_cxt.periodic.rate=1")

def main():
    manager = SoarRTCManager()
    manager.start()

if __name__=='__main__':
    main()
