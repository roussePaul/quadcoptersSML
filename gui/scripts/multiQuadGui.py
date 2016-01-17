#!/usr/bin/env python

import sys

from gui.multiQuadGui import multiQuadGui
from rqt_gui.main import Main

plugin = 'multiQuadGui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
