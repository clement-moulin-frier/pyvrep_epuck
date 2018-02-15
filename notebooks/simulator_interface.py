import sys
sys.path.append('../..')
sys.path.append('../../pypot')
sys.path.append('../../enum34')
sys.path.append('../../pyserial-3.4')

from functools import partial

from pypot.vrep import close_all_connections
from pyvrep_epuck.vrep.simulator import get_session as gs
from pyvrep_epuck.vrep.simulator import close_session


get_session = partial(gs, n_epucks=1, use_proximeters=[1, 4])

open_session = get_session