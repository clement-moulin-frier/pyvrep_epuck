import sys
from os.path import abspath
sys.path.append('../..')
sys.path.append('../../pypot')
print(sys.path)

from pypot.vrep import close_all_connections
from pyvrep_epuck.vrep.simulator import get_session

