#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''SEAT (Speech Event Action Transfer)

Copyright (C) 2009-2010
    Yosuke Matsusaka and Isao Hara
    Intelligent Systems Research Institute,
    National Institute of Advanced Industrial Science and Technology (AIST),
    Japan
    All rights reserved.
Licensed under the Eclipse Public License -v 1.0 (EPL)
http://www.opensource.org/licenses/eclipse-1.0.txt
'''

import sys
import os
import getopt
import codecs
import locale
import time
import signal
import re
import traceback
import socket
import optparse
import threading
import OpenRTM_aist
import RTC
from lxml import etree
from BeautifulSoup import BeautifulSoup

from Tkinter import *

from seatsat.__init__ import __version__
from seatsat import utils
try:
    import gettext
    _ = gettext.translation(domain='seatsat', localedir=os.path.dirname(__file__)+'/../share/locale').ugettext
except:
    _ = lambda s: s

__doc__ = _('''\
SEAT(Speech Event Action Transfer) is a simple dialog manager for robotic applications.
The interactive behavior of the system can be realized without complex programming.

SEAT has following features:
 1. Paraphrase matching function.
 2. Conversation management function based on state transition model.
 3. Adapter functions (supports OpenRTM , BSD socket, etc...).
''')

class SocketAdaptor(threading.Thread):
    def __init__(self, seat, name, host, port):
        threading.Thread.__init__(self)
        self.seat = seat
        self.name = name
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connected = False
        self.mainloop = True
        self.start()

    def run(self):
        while self.mainloop:
            if self.connected == False:
                try:
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.socket.connect((self.host, self.port))
                    self.socket.settimeout(1)
                    self.connected = True
                except socket.error:
                    print "reconnect error in ", self.name
                    time.sleep(1)
                except:
                    print traceback.format_exc()
            if self.connected == True:
                try:
                    data = self.socket.recv(1024)
                    if len(data) != 0:
                        self.seat.processResult(self.name, data)
                except socket.timeout:
                    pass
                except socket.error:
                    self.socket.close()
                    self.connected = False
                    print traceback.format_exc()
                except:
                    print traceback.format_exc()

    def terminate(self):
        self.mainloop = False
        if self.connected == True:
            self.socket.close()
            self.connected = False
        
    def send(self, name, msg):
        if self.connected == False:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.host, self.port))
                self.connected = True
                print "connect socket"
            except socket.error:
                print "cannot connect"
        if self.connected == True:
            try:
                self.socket.sendall(msg+"\n")
            except socket.error:
                #print traceback.format_exc()
                self.socket.close()
                self.connected = False
                print traceback.format_exc()

seat_spec = ["implementation_id", "SEAT",
             "type_name",         "SEAT",
             "description",       __doc__.encode('UTF-8'),
             "version",           __version__,
             "vendor",            "Yosuke Matsusaka and Isao Hara, AIST",
             "category",          "Speech",
             "activity_type",     "DataFlowComponent",
             "max_instance",      "1",
             "language",          "Python",
             "lang_type",         "script",
             "conf.default.scriptfile", "none",
             "conf.__description__.scriptfile", _("Script file to load (unimplemented).").encode('UTF-8'),
             "conf.default.scorelimit", "0.0",
             "conf.__widget__.scorelimit", "slider",
             "conf.__description__.scorelimit", _("Lower limit of speech recognition score to consider.").encode('UTF-8'),
             "exec_cxt.periodic.rate", "100.0",
             ""]

class DataListener(OpenRTM_aist.ConnectorDataListenerT):
    def __init__(self, name, type, obj):
        self._name = name
        self._type = type
        self._obj = obj
    
    def __call__(self, info, cdrdata):
        data = OpenRTM_aist.ConnectorDataListenerT.__call__(self, info, cdrdata, self._type(RTC.Time(0,0),None))
        self._obj.onData(self._name, data)


class SEAT(OpenRTM_aist.DataFlowComponentBase):
    def __init__(self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        if hasattr(sys, "frozen"):
            self._basedir = os.path.dirname(unicode(sys.executable, sys.getfilesystemencoding()))
        else:
            self._basedir = os.path.dirname(__file__)
        xmlschema_doc = etree.parse(os.path.join(self._basedir, 'seatml.xsd'))
        self._xmlschema = etree.XMLSchema(xmlschema_doc)
        self.states = []
        self.currentstate = "start"
        self.keys = {}
        self.regkeys = {}
        self.adaptors = {}
        self.adaptortype = {}
        self.statestack = []
        self._data = {}
        self._port = {}
        self._scriptfile = ["none"]
        self._scorelimit = [0.0]
        self.max_score = 0
        self.gui_flag = False
        self.init_state = None
        self.gui_buttons = {}
        self.frames = {}
        self.root = Tk()

    def onInitialize(self):
        OpenRTM_aist.DataFlowComponentBase.onInitialize(self)
        self._logger = OpenRTM_aist.Manager.instance().getLogbuf(self._properties.getProperty("instance_name"))
        self._logger.RTC_INFO("SEAT (Speech Event Action Transfer) version " + __version__)
        self._logger.RTC_INFO("Copyright (C) 2009-2010 Yosuke Matsusaka and Isao Hara")
        self.bindParameter("scriptfile", self._scriptfile, "none", self.scriptfileTrans)
        self.bindParameter("scorelimit", self._scorelimit, "0.0")
        return RTC.RTC_OK

    def onFinalize(self):
        OpenRTM_aist.DataFlowComponentBase.onFinalize(self)
        try:
            for a in self.adaptors.itervalues():
                if isinstance(a, SocketAdaptor):
                    a.terminate()
                    a.join()
        except:
            self._logger.RTC_ERROR(traceback.format_exc())
        return RTC.RTC_OK

    def scriptfileTrans(self, _type, _str): 
        # self._logger.RTC_INFO("scriptfile = " + _str)
        if _str != "none":
            try:
                self.loadSEATML(_str.split(','))
            except:
                self._logger.RTC_ERROR(traceback.format_exc())
        return OpenRTM_aist.stringTo(_type, _str)

    def createInPort(self, name, type=RTC.TimedString):
        self._logger.RTC_INFO("create inport: " + name)
        self._data[name] = type(RTC.Time(0,0), None)
        self._port[name] = OpenRTM_aist.InPort(name, self._data[name])
        self._port[name].addConnectorDataListener(OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE,
                                                  DataListener(name, type, self))
        self.registerInPort(name, self._port[name])

    def createOutPort(self, name, type=RTC.TimedString):
        self._logger.RTC_INFO("create outport: " + name)
        self._data[name] = type(RTC.Time(0,0), None)
        self._port[name] = OpenRTM_aist.OutPort(name, self._data[name], OpenRTM_aist.RingBuffer(8))
        self.registerOutPort(name, self._port[name])

    def onData(self, name, data):
        try:
            if isinstance(data, RTC.TimedString):
                data.data = data.data.decode('utf-8')
                self.processResult(name, data.data)
            else:
                self.processNonString(name, data.data)
        except:
            self._logger.RTC_ERROR(traceback.format_exc())

    def onExecute(self, ec_id):
        OpenRTM_aist.DataFlowComponentBase.onExecute(self, ec_id)
        return RTC.RTC_OK

    def send(self, name, data):
        if isinstance(data, str) :
            self._logger.RTC_INFO("sending command %s (to %s)" % (data, name))
        else:
            self._logger.RTC_INFO("sending command to %s" % (name,))

        dtype = self.adaptortype[name][1]
        if dtype == str:
            ndata = dtype(data.encode('utf-8'))
        elif self.adaptortype[name][2]:
            ndata = []
            for d in data.split(","):
                ndata.append(dtype(d))
        else:
            ndata = dtype(data)
        self._data[name].data = ndata
        self._port[name].write()

    def processResult(self, host, s):
        global rtc_in_data
        try:
            s = unicode(s)
        except UnicodeDecodeError:
            s = str(s).encode('string_escape')
            s = unicode(s)
        self._logger.RTC_INFO("got input %s (%s)" % (s, host))

        cmds = None
        if s.count('<?xml') > 0:
            doc = BeautifulSoup(s)
            for s in doc.findAll('data'):
                rank = int(s['rank'])
                score = float(s['score'])
                text = s['text']
                if score < self.max_score:
                    break
                self._logger.RTC_INFO("#%i: %s (%f)" % (rank, text, score))
                if score < self._scorelimit[0]:
                    self._logger.RTC_INFO("[rejected] score under limit")
                    continue
                cmds = self.lookupwithdefault(self.currentstate, host, text)
                if not cmds:
                    cmds = self.lookupwithdefault(self.currentstate, host, text)
                if cmds:
                    break
                else:
                    self._logger.RTC_INFO("[rejected] no matching phrases")
            rtc_in_data = text
        else:
            cmds = self.lookupwithdefault(self.currentstate, host, s)
            rtc_in_data = s

        if not cmds:
            self._logger.RTC_INFO("no command found")
            return False

        for c in cmds:
            self.activateCommand(c)
        return True

    def processNonString(self, host, s):
        self._logger.RTC_INFO("got input from %s" %  (host,))
        cmds = self.lookupwithdefault(self.currentstate, host, host)
        if not cmds:
            self._logger.RTC_INFO("no command found")
            return False
        for c in cmds:
            self.activateCommandEx(c, s)
        return True

    def lookupwithdefault(self, state, host, s):
        self._logger.RTC_INFO('looking up...%s: %s' % (host,s,))
        cmds = self.lookupcommand(state, host, s)
        if not cmds:
            cmds = self.lookupcommand(state, 'default', s)
        if not cmds:
            cmds = self.lookupcommand('all', host, s)
        if not cmds:
            cmds = self.lookupcommand('all', 'default', s)
        return cmds

    def lookupcommand(self, state, host, s):
        cmds = []
        regkeys = []
        try:
            cmds = self.keys[state+":"+host+":"+s]
        except KeyError:
            try:
                regkeys = self.regkeys[state+":"+host]
            except KeyError:
                return None
            for r in regkeys:
                if r[0].match(s):
                    cmds = r[1]
                    break
            return None
        return cmds
        

    def stateTransfer(self, newstate):
        try:
            for c in self.keys[self.currentstate+":::exit"]:
                self.activateCommand(c)
        except KeyError:
            pass
        self.currentstate = newstate
        try:
            for c in self.keys[self.currentstate+":::entry"]:
                self.activateCommand(c)
        except KeyError:
            pass

    def activateCommand(self, c):
        if c[0] == 'c':
            host = c[1]
            data = c[2]
            try:
                ad = self.adaptors[host]
                ad.send(host, data)
            except KeyError:
                self._logger.RTC_ERROR("no such adaptor:" + host)
        elif c[0] == 't':
            func = c[1]
            data = c[2]
            if (func == "push"):
                self.statestack.append(self.currentstate)
                self.stateTransfer(data)
            elif (func == "pop"):
                if self.statestack.__len__() == 0:
                    self._logger.RTC_WARN("state buffer is empty")
                    return
                self.stateTransfer(self.statestack.pop())
            else:
                self._logger.RTC_INFO("state transition from "+self.currentstate+" to "+data)
                self.hide_frame(self.currentstate)
                self.show_frame(data)
                self.stateTransfer(data)
        elif c[0] == 'l':
            data = c[1]
            self._logger.RTC_INFO(data)

        elif c[0] == 'x':
            host = c[1]
            data = c[2]
            res = os.system(data)
            try:
                ad = self.adaptors[host]
                ad.send(host, res)
            except KeyError:
                self._logger.RTC_ERROR("no such adaptor:")

        elif c[0] == 's':
            rtc_result = None
            host = c[1]
            data = c[2]
            exec(data)
            if rtc_result == None :
              pass
            else:
                try:
                    ad = self.adaptors[host]
                    ad.send(host, rtc_result)
                except KeyError:
                    self._logger.RTC_ERROR("no such adaptor:")

    def activateCommandEx(self, c, s):
        if c[0] == 'c':
            host = c[1]
            data = c[2]
            try:
                ad = self.adaptors[host]
                ad.send(host, data)
            except KeyError:
                self._logger.RTC_ERROR("no such adaptor:" + host)

        elif c[0] == 't':
            func = c[1]
            data = c[2]
            if (func == "push"):
                self.statestack.append(self.currentstate)
                self.stateTransfer(data)
            elif (func == "pop"):
                if self.statestack.__len__() == 0:
                    self._logger.RTC_WARN("state buffer is empty")
                    return
                self.stateTransfer(self.statestack.pop())
            else:
                self._logger.RTC_INFO("state transition from "+self.currentstate+" to "+data)
                self.stateTransfer(data)

        elif c[0] == 'l':
            data = c[1]
            self._logger.RTC_INFO(data)

        elif c[0] == 'x':
            host = c[1]
            data = c[2]
            res = os.system(data)
            try:
                ad = self.adaptors[host]
                ad.send(host, res)
            except KeyError:
                self._logger.RTC_ERROR("no such adaptor:" + host)

        elif c[0] == 's':
            rtc_result = None
            host = c[1]
            data = c[2]

            rtc_in_data = s
            exec(data)
            if rtc_result == None :
              pass
            else:
                try:
                    ad = self.adaptors[host]
                    ad.send(host, rtc_result)
                except KeyError:
                    self._logger.RTC_ERROR("no such adaptor:" + host)

    def getDataType(self, s):
        if len(s) == 0:
            return (RTC.TimedString, 0)
        seq = False
        if s[-3:] == "Seq":
            seq = True
        dtype = str
        if s.count("WString"):
            dtype = unicode
        elif s.count("Float"):
            dtype = float
        elif s.count("Double"):
            dtype = float
        elif s.count("Short"):
            dtype = int
        elif s.count("Long"):
            dtype = int
        elif s.count("Octet"):
            dtype = int
        elif s.count("Char"):
            dtype = int
        elif s.count("Boolean"):
            dtype = int
        return (eval("RTC.%s" % s), dtype, seq)

    def parsecommands(self, r):
        commands = []
        for c in r.findall('command'): # get commands
            host = c.get('host')
            data = c.text
            commands.append(['c', host, data])
        for c in r.findall('statetransition'): # get statetransition (as command)
            func = c.get('func')
            data = c.text
            commands.append(['t', func, data])
        for c in r.findall('log'): # get statetransition (as command)
            data = c.text
            commands.append(['l', data])
        for c in r.findall('shell'): # get shell (as command)
            func = c.get('host')
            data = c.text
            commands.append(['x', func, data])
        for c in r.findall('script'): # get shell (as command)
            func = c.get('host')
            data = c.text
            commands.append(['s', func, data])
        return commands

    def loadSEATML(self, files):
        for f in files:
            f = f.replace("\\", "\\\\")
            self._logger.RTC_INFO(u"load script file: " + f)
            try:
                doc = etree.parse(f)
            except etree.XMLSyntaxError, e:
                self._logger.RTC_ERROR(u"invalid xml syntax: " + unicode(e))
                continue
            except IOError, e:
                self._logger.RTC_ERROR(u"unable to open file " + f + ": " + unicode(e))
                continue
            try:
                self._xmlschema.assert_(doc)
            except AssertionError, b:
                self._logger.RTC_ERROR(u"invalid script file: " + f + ": " + unicode(b))
                continue

            self.buttons = {}

            for g in doc.findall('general'):
                for a in g.findall('agent'):
                    name = str(a.get('name'))
                    type = a.get('type')
                    if type == 'rtcout':
                        self.adaptortype[name] = self.getDataType(a.get('datatype'))
                        self.createOutPort(name, self.adaptortype[name][0])
                        self.adaptors[name] = self
                    elif type == 'rtcin':
                        self.adaptortype[name] = self.getDataType(a.get('datatype'))
                        self.createInPort(name, self.adaptortype[name][0])
                        self.adaptors[name] = self
                    else:
                        host = a.get('host')
                        port = int(a.get('port'))
                        self.adaptors[name] = SocketAdaptor(self, name, host, port)

            for s in doc.findall('state'):
                name = s.get('name')
                self.frames[name] = Frame(self.root)
                self.buttons[name]=[]
                if self.init_state == None:
                    self.init_state = name

                for e in s.findall('onentry'):
                    commands = self.parsecommands(e)
                    self._logger.RTC_INFO("register "+name+":::entry")
                    self.keys[name+":::entry"] = commands # register commands to key table
                for e in s.findall('onexit'):
                    commands = self.parsecommands(e)
                    self._logger.RTC_INFO("register "+name+":::exit")
                    self.keys[name+":::exit"] = commands # register commands to key table
                for r in s.findall('rule'):
                    words = []
                    commands = self.parsecommands(r)
                    for k in r.findall('key'): # get keys
                        source = k.get('source')
                        word = self.decompString([k.text])
                        if source is None:
                            words.extend(word)
                        else:
                            for w in word:
                                self._logger.RTC_INFO("register "+name+":"+source+":"+w)
                                self.keys[name+":"+source+":"+w] = commands # register commands to key table
                                self.buttons[name].append(w)

                    for k in r.findall('regkey'): # get keys
                        try:
                            regkeys = self.regkeys[name+":default"]
                        except KeyError:
                            regkeys = []
                        regkeys.append([re.compile(k.text), commands])
                        self.regkeys[name+":default"] = regkeys

                    for w in words:
                        self._logger.RTC_INFO("register " + name + ":default:" + w)
                        self.keys[name+":default:"+w] = commands # register commands to key table
                        self.buttons[name].append(w)

                self.states.extend([name])

        if len(self.states) == 0:
            self._logger.RTC_ERROR("no available state")
            return 1
        self.startstate = None
        if self.states.count("start") > 0:
            self.startstate = "start"
        else:
            self.startstate = self.states[0]
        self.stateTransfer(self.startstate)
        self._logger.RTC_INFO("current state " + self.currentstate)
        self._logger.RTC_INFO("loaded successfully")
        return 0

    def decompString(self, strs):
        ret = []
        nstrs = strs
        while nstrs.__len__() > 0:
            nstrs2 = []
            for str in nstrs:
                if str.count('(') > 0 or str.count('[') > 0:
                    nstrs2.extend(self.decompStringSub(str))
                else:
                    ret.extend([str])
            nstrs = nstrs2
        return ret

    def decompStringSub(self, str):
        ret = []
        bc = str.count('(')
        kc = str.count('[')
        if bc > 0:
            i = str.index('(')
            prestr = str[:i]
            substrs = []
            substr = ''
            level = 0
            i += 1
            while i < str.__len__():
                if str[i] == '(':
                    level += 1
                    substr += str[i]
                elif str[i] == ')':
                    if level == 0:
                        substrs.extend([substr])
                        break
                    else:
                        substr += str[i]
                    level -= 1
                elif str[i] == '|':
                    if level == 0:
                        substrs.extend([substr])
                        substr = ''
                    else:
                        substr += str[i]
                else:
                    substr += str[i]
                i += 1
            poststr = str[i+1:]
            for s in substrs:
                ret.extend([prestr+s+poststr])
        elif kc > 0:
            i = str.index('[')
            prestr = str[:i]
            substr = ''
            level = 0
            i += 1
            while i < str.__len__():
                if str[i] == '[':
                    level += 1
                elif str[i] == ']':
                    if level == 0:
                        break
                    level -= 1
                substr += str[i]
                i += 1
            poststr = str[i+1:]
            ret.extend([prestr+poststr])
            ret.extend([prestr+substr+poststr])
        else:
            ret.extend([str])
        return ret


    def mkcallback(self, name):
        def __callback_func__():
           self.processResult("gui", name)
        return __callback_func__

    def create_button(self, frame, name):
        btn = Button(frame, text=name, command=self.mkcallback(name) )
        return btn

    def pack_buttons(self, name):
        n=10
        if self.gui_buttons[name] :
           i=0
           j=0
           for b in self.gui_buttons[name] :
               if ( i % 10 ) == 0:
                   j += 1
               b.grid(row=j, column=i, sticky=W + E)
               i = (i+1) % 10


    def show_frame(self, name):
        if self.frames[name] :
           self.frames[name].place(relx=0.0, rely=0, relwidth=1, relheight=1)

    def hide_frame(self, name):
        if self.frames[name] :
           self.frames[name].place_forget()

    def create_gui(self, name):
        if name:
           buttons = self.buttons[name]
           self.gui_buttons[name] = []
           for b in buttons:
               self.gui_buttons[name].append( self.create_button(self.frames[name],b))
           self.pack_buttons(name)

        return 0


class SEATManager:
    def __init__(self):
        global opts
        encoding = locale.getpreferredencoding()
        sys.stdout = codecs.getwriter(encoding)(sys.stdout, errors = "replace")
        sys.stderr = codecs.getwriter(encoding)(sys.stderr, errors = "replace")
        
        parser = utils.MyParser(version=__version__, usage="%prog [seatmlfile]",
                                description=__doc__)
        utils.addmanageropts(parser)
        parser.add_option('-g', '--gui', dest='guimode', action="store_true",
                          default=False,
                          help=_('show file open dialog in GUI'))

        parser.add_option('-t', '--test_mode', dest='testmode', action="store_true",
                          default=False,
                          help=_('show GUI panel for test'))

        parser.add_option('-s', type='float', nargs=1, dest='maxscore',
                          default=0.0,
                          help=_('max_score for voice recognition'))
        try:
            opts, args = parser.parse_args()
        except optparse.OptionError, e:
            print >>sys.stderr, 'OptionError:', e
            sys.exit(1)

        if opts.guimode == True:
            sel = utils.askopenfilenames(title="select script files")
            if sel is not None:
                args.extend(sel)
    
        if len(args) == 0:
            parser.error("wrong number of arguments")
            sys.exit(1)

        self._scriptfiles = args
        self.comp = None
        self.manager = OpenRTM_aist.Manager.init(utils.genmanagerargs(opts))
        self.manager.setModuleInitProc(self.moduleInit)
        self.manager.activateManager()

    def start(self):
        global opts
        if opts.testmode:
            self.manager.runManager(True)
            for st in self.comp.states:
                self.comp.create_gui(st)
            self.comp.show_frame(self.comp.init_state)
            self.comp.root.mainloop()
        else:
            self.manager.runManager(False)
        #if opts.guimode == True:
        #    raw_input("Press Enter to Exit")

    def moduleInit(self, manager):
        profile = OpenRTM_aist.Properties(defaults_str=seat_spec)
        manager.registerFactory(profile, SEAT, OpenRTM_aist.Delete)
        self.comp = manager.createComponent("SEAT?exec_cxt.periodic.rate=1")
        if opts.testmode == True:
            self.comp.gui_flag = True
        self.comp.max_score = opts.maxscore
        ret = self.comp.loadSEATML(self._scriptfiles)
        if ret != 0:
            print >>sys.stderr, 'Unable to load script file: see log file for details...'

def main():
    seat = SEATManager()
    seat.start()
    return 0

if __name__=='__main__':
    sys.exit(main())

