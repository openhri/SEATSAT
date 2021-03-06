#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''visualization tool for SEAT (Speech Event Action Transfer)

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
import codecs
import optparse
import locale
from seatsat.__init__ import __version__
from xml.dom.minidom import parse
from seatsat import utils
try:
    import gettext
    _ = gettext.translation(domain='seatsat', localedir=os.path.dirname(__file__)+'/../share/locale').ugettext
except:
    _ = lambda s: s

__doc__ = _('Draw graph from SEAT script file.')

__examples__ = '''
Examples:

- '''+_('Draw graph of the SEAT script file.')+'''

  ::
  
  $ seatmltographviz sample.seatml | dot -Txlib
'''

def seatmltographviz(doc):
    ret = ""

    ret += 'digraph seatml {\n'
    ret += 'graph [rankdir=LR];\n'
    
    states = map(lambda x:x.getAttribute('name'), doc.getElementsByTagName('state'))
    states.extend(map(lambda x:x.childNodes[0].data, doc.getElementsByTagName('statetransition')))
    ret += 'node [shape = doublecircle]; %s;\n' % ' '.join(states)
    ret += 'node [shape = circle];\n'
    
    for s in doc.getElementsByTagName('state'):
        for r in s.getElementsByTagName('rule'):
            lab = '[label = "'
            #lab = lab + ",".join(map(lambda x: x.childNodes[0].data, r.getElementsByTagName('key')))
            source = r.getElementsByTagName('key').item(0).getAttribute('source')
            if source:
                lab = lab + source + "="
            lab = lab + r.getElementsByTagName('key').item(0).childNodes[0].data
            lab = lab + '"]'
            t = r.getElementsByTagName('statetransition')
            if len(t) > 0:
                ret += "%s -> %s %s;\n" % (s.getAttribute('name'), t.item(0).childNodes[0].data, lab)
            else:
                ret += "%s -> %s %s;\n" % (s.getAttribute('name'), s.getAttribute('name'), lab)
    ret += '}\n'
    return ret

def main():
    encoding = locale.getpreferredencoding()
    sys.stdout = codecs.getwriter(encoding)(sys.stdout, errors = "replace")
    sys.stderr = codecs.getwriter(encoding)(sys.stderr, errors = "replace")

    parser = utils.MyParser(version=__version__, usage="%prog [seatmlfile]",
                            description=__doc__, epilog=__examples__)
    parser.add_option('-v', '--verbose', dest='verbose', action='store_true',
                      default=False,
                      help=_('output verbose information'))
    try:
        opts, args = parser.parse_args()
    except optparse.OptionError, e:
        print >>sys.stderr, 'OptionError:', e
        sys.exit(1)

    if len(args) == 0:
        parser.error("wrong number of arguments")
        sys.exit(1)

    doc = parse(args[0])
    print seatmltographviz(doc)

if __name__ == '__main__':
    main()

