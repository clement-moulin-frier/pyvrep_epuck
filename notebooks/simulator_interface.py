import sys
sys.path.append('../..')
sys.path.append('../../pypot')
sys.path.append('../../enum34')

from functools import partial

from pypot.vrep import close_all_connections
from pyvrep_epuck.vrep.simulator import get_session as gs


get_session = partial(gs, n_epucks=1, use_proximeters=[2, 3])