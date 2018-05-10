import logging
import logging.handlers
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie

from threading import Thread

sigTones = [5000, 7000, 11000, 13000, 15000]
chirpParams = [[10000, 2000], [14000, -1000]]
chirpLen = 500

nTones = len(sigTones)
nChirps = len(chirpParams)
dutyCycle = 10000
nCopters = 1
nFreqs = 2
doChirp = True

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class CrazyflieManager:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """


    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()
        self.logger = logging.getLogger(__name__)
        self.logger.addHandler(logging.StreamHandler())

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self.logger.info('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = False
        self.connection_failed = False

        self._param_check_list = []
        self._param_groups = []
        self.params_set = {}
        self._isBusy = False
        self._willStop = False
        self._currentParam = None


        self._nextThrottle = 0 # TODO: next pitch/roll/yaw etc

        #self.params_set = {'mtrsnd.f1':[0,f1], 'mtrsnd.f2':[0,f2], 
        #        'mtrsnd.f3':[0,f3]}

    def stop(self):
        self._willStop = True


    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        self.logger.info('Connected to %s' % link_uri)
        self.is_connected = True
        self.logger.debug('registering parameter callback')
        p_toc = self._cf.param.toc.toc
        group = 'mtrsnd'
        self.logger.debug('{}'.format(group))
        for param in sorted(p_toc[group].keys()):
            self.logger.debug('\t{}'.format(param))
            self._param_check_list.append('{0}.{1}'.format(group, param))
        self._param_groups.append('{}'.format(group))
        # register update callback for mtrsnd
        self._cf.param.add_update_callback(group='mtrsnd', name=None,
                                           cb=self._param_callback)

        self.logger.debug('')
        Thread(target=self._main_loop).start()

    def update_parameters(self, paramName, paramVal):
        # add to the dictionary whether we're busy or not 
        # TODO: i guess work like parameter update can go on a queue
        newParams = dict((k, [-1, v]) for (k,v) in zip(paramName, paramVal))
        self.params_set.update(newParams)
        self._isBusy = True
        

    def _main_loop(self):
        # set each of the desired params
        # if there is nothing left to set, turn off the busy flag

        while not self._willStop:
            if self._currentParam is None:
                finished = True
                for p,opts in self.params_set.items():
                    if str(opts[0]) != str(opts[1]):
                        # TODO: we need to use the cflib info like type to do this check for real
                        finished = False
                        self.logger.debug('{} is {}, setting to {}'.format(p, opts[0], opts[1]))
                        self._cf.param.set_value(p, str(opts[1]))
                        self._currentParam = p
                        break

                if finished:
                    self._isBusy = False

            self._cf.commander.send_setpoint(0,0,0,self._nextThrottle)
            time.sleep(0.025)

    def _param_callback(self, name, value):
        """Generic callback registered for all the groups"""
        if name in self.params_set.keys():
            self.params_set[name][0] = value
        if name == self._currentParam:
            self._currentParam = None
            self.logger.debug('{} set to {}'.format(name, value))


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        self.logger.error('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False
        self.connection_failed = True

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        self.logger.error('Connection to %s lost: %s' % (link_uri, msg))
        self.connection_failed = True

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        self.logger.warning('Disconnected from %s' % link_uri)
        self.is_connected = False
        self._willStop = True


def setFreqAndWait(cf, n, freq):
    paramName = "mtrsnd.f{}".format(n+1)
    cf.update_parameters([paramName], [freq])
    while(pe._isBusy):
        time.sleep(.1)

def setChirpParams(start, end):
    pe.update_parameters(["mtrsnd.chirpF1", "mtrsnd.chirpF2"], [start, end])
    while(pe._isBusy):
        time.sleep(.1)


def startChirp():
    pe.update_parameters(['mtrsnd.goChirp'], [1])
    while(pe._isBusy):
        time.sleep(.1)


if __name__ == '__main__':
    import sys
    addr = None
    logger = logging.getLogger('MAIN')
    logger.addHandler(logging.StreamHandler())

    done = False
    
    urlFormat = "radio://0/80/250K/E{}E7E7E7E7"
    selectedCopters = [2,3]

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

   # cflies = []
   # for n in selectedCopters:
   #     copterName = urlFormat.format(n)
   #     try:
   #         pe = CrazyFlieManager(copterName)
   #         cflies.append(pe)
   #     except:
   #         logger.warning("Could not create controller for {}".format(copterName))

   # nFlies = len(cflies)
   # nConnected = 0

   # while nConnected < nFlies:
   #     for cf in cflies:
   #         if cf.is_connected or cf.connection_failed: nConnected = nConnected+1        
   # 

   # cflies = [cf for cf in cflies if cf.is_connected]
    pe = CrazyflieManager(urlFormat.format(2));

    
    for i in range(10):
        pe._nextThrottle = 0
        time.sleep(.050)

    if doChirp:
        idxs = range(nChirps)
    else:
        idxs = range(nTones)
        
    # first the tones
    for t in idxs:
        setFreqAndWait(pe, 0, 0)
        setFreqAndWait(pe, 1, 0)
        setFreqAndWait(pe, 2, 0)
        setFreqAndWait(pe, 3, 0)
        
        pe._nextThrottle = dutyCycle;
        if doChirp:
            pe.update_parameters(['mtrsnd.f1', 'mtrsnd.f2', 'mtrsnd.chpF1', 'mtrsnd.chpF2', 'mtrsnd.chpLen'], [10000, 14000, 12000, 13000, 1000]);
            #setChirpParams(cp[0], cp[1])

        for i in range(20):
            time.sleep(.05)

        if doChirp:
            startChirp()
        else:
            f = sigTones[t]
            setFreqAndWait(pe, 0, f)
            setFreqAndWait(pe, 1, f)
        
        time.sleep(chirpLen)

        setFreqAndWait(pe, 0, 0)
        setFreqAndWait(pe, 1, 0)

        for i in range(20):
            time.sleep(.050)
        pe._nextThrottle = 0

        for i in range(20):
            time.sleep(.050)

        logger.info('Shutting down')

        pe._nextThrottle = 0
        time.sleep(1)
        pe._cf.close_link()
        # End the example by closing the link (will cause the app to quit)
    else:
        logger.error('No Crazyflies found, cannot run example')
