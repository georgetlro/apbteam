import asserv
import asserv.init
import mimot
import mimot.init
import io_hub
import io_hub.init

from proto.popen_io import PopenIO
import math
import subprocess

class Robot:
    """APBirthday robot instance."""

    import simu.model.table_eurobot2013 as table_model
    import simu.view.table_eurobot2013 as table_view

    import simu.robots.apbirthday.link.bag as robot_link
    import simu.robots.apbirthday.model.bag as robot_model
    import simu.robots.apbirthday.view.bag as robot_view

    robot_start_pos = {
            False: (650, 2000 - 250, math.radians (90)),
            True: (3000 - 650, 2000 - 250, math.radians (90))
            }

    client_nb = 4

    tick = 1000

    def __init__ (self, proto_time, instance = 'robot0'):
        self.instance = instance
        def proto (proto_class, cmd, init):
            cmd = [ s.format (instance = instance) for s in cmd ]
            return proto_class (PopenIO (cmd), proto_time, **init)
        def prog (cmd):
            cmd = [ s.format (instance = instance) for s in cmd ]
            subprocess.Popen (cmd)
        asserv_cmd = ('../../mimot/src/asserv/asserv.host',
                '-i{instance}:asserv0', '-m9', 'apbirthday')
        mimot_cmd = ('../../mimot/src/dirty/dirty.host',
                '-i{instance}:mimot0', '-m9', 'apbirthday')
        io_hub_cmd = ('../../io-hub/src/apbirthday/apbirthday.host',
                '-i{instance}:io0')
        beacon_stub_cmd = ('../../beacon/src/stub/simu_stub.host',
                '-i{instance}:beacon0')
        self.asserv = proto (asserv.Proto, asserv_cmd,
                asserv.init.host['apbirthday'])
        self.mimot = proto (mimot.Proto, mimot_cmd,
                mimot.init.host['guybrush'])
        self.io = proto (io_hub.ProtoGuybrush, io_hub_cmd,
                io_hub.init.host['apbirthday'])
        self.beacon_stub = prog (beacon_stub_cmd)
        self.protos = (self.asserv, self.mimot, self.io)

    def jack (self):
        if self.model.jack.state:
            plate_pos = ((200, 1400), (3000 - 200, 1400))[
                    self.model.color_switch.state]
            plate = self.model.table.nearest (plate_pos, max = 1)
            if plate:
                plate.pos = None
                plate.notify ()
