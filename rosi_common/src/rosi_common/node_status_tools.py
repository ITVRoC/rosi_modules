
from rosi_common.srv import SetNodeStatusResponse, GetNodeStatusListResponse

'''Class that manages node status'''
class nodeStatus():

    def __init__(self, node_name):
        '''Class constructor'''

        ##=== Useful variables

        # possible status
        self.states = (
            'active',
            'bypass',
            'haltcmd',
            'telemetry'
        )

        # starting node status. All nodes starts in idle mode
        self.nodeStatus = {
            self.states[0]: True,   # active
            self.states[1]: False,  # bypass
            self.states[2]: False,  # halt
            self.states[3]: True    # telemetry
        }

        # node name
        self.node_name = node_name


    def getPossibleStates(self):
        '''Returns possible node states'''
        return self.states


    def getNodeStatus(self):
        '''Returns node current status'''
        return self.nodeStatus


    def getNodeStatusList(self):
        '''Returns node current status in list mode'''
        return [val for val in self.nodeStatus.values()]


    def getNodeStatusSrvResponse(self):
        ''' Returns a service message containing current node status list'''
        s = GetNodeStatusListResponse()
        s.node_status = self.getNodeStatusList()
        return s

    ''' === Methods for defining flags considering a ROS service input ==='''

    def defServiceReq(self, req, type, setMethod, resetMethod, rospy):
        '''Base function that defines a node status flag and returns the response as a ROS service message
        It receives a setMethod and resetMethod for performning its action.'''

         # response variable
        response = False

        # setting or reseting active flag considering received request
        if req.set_value:
            setMethod()
            if self.getNodeStatus()[type]:
                response = True

        elif not req.set_value:
            resetMethod()
            if not self.getNodeStatus()[type]:
                response = True
        else:
            rospy.logerr('Method setServiceReq received a bad request.')

        # creating message to return
        s = SetNodeStatusResponse()
        s.return_status = response

        return s


    def defActiveServiceReq(self, req, rospy):
        '''Defines de Active flag by service call '''
        return self.defServiceReq(req, 'active', self.setActive, self.resetActive, rospy)


    def defBypassServiceReq(self, req, rospy):
        '''Defines de Bypass flag by service call '''
        return self.defServiceReq(req, 'bypass', self.setBypass, self.resetBypass, rospy)


    def defHaltCmdServiceReq(self, req, rospy):
        '''Defines de HaltCmd flag by service call '''
        return self.defServiceReq(req, 'haltcmd', self.setHaltCmd, self.resetHaltCmd, rospy)


    def defTelemetryServiceReq(self, req, rospy):
        '''Defines de Telemetry flag by service call '''
        return self.defServiceReq(req, 'telemetry', self.setTelemetry, self.resetTelemetry, rospy)
       

    ''' === SETTERS AND GETTERS === '''    
    def setActive(self):
        '''Sets the node in active state
        When you activate a node, telemetry automatically activates'''
        self.nodeStatus['active'] = True
        self.nodeStatus['telemetry'] = True

    def resetActive(self):
        '''Resets the active flag
        When you disable a node, telemetry disables automatically'''
        self.nodeStatus['active'] = False
        self.nodeStatus['telemetry'] = False

    def setBypass(self):
        '''Sets the node in bypass mode'''
        self.nodeStatus['bypass'] = True

    def resetBypass(self):
        '''Resets the node in bypass mode'''
        self.nodeStatus['bypass'] = False

    def setHaltCmd(self):
        '''Sets the node in HaldCmd mode'''
        self.nodeStatus['haltcmd'] = True
        
    def resetHaltCmd(self):
        '''Resets the node in HaldCmd mode'''
        self.nodeStatus['haltcmd'] = False

    def setTelemetry(self):
        '''Sets the node in telemetry mode'''
        self.nodeStatus['telemetry'] = True

    def resetTelemetry(self):
        '''Resets the node in telemetry mode'''
        self.nodeStatus['telemetry'] = False
        
    def resetNode(self):
        '''Reset all node status '''
        self.nodeStatus = {
            self.states[0]: True,
            self.states[1]: False,
            self.states[2]: False,
            self.states[3]: True
        }


    def getSrvPath(self, type, rospy):
        '''Returns the service path given its type'''

        if type == self.states[0]: #active
            return '/node_status/'+self.node_name+'/def_active' 

        elif type == self.states[1]: #bypass
            return '/node_status/'+self.node_name+'/def_bypass' 

        elif type == self.states[2]: #haltcmd
            return '/node_status/'+self.node_name+'/def_haltcmd' 

        elif type == self.states[3]: #telemetry
            return '/node_status/'+self.node_name+'/def_telemetry' 

        elif type == 'getNodeStatus': # get node status
            return '/node_status/'+self.node_name+'/get_node_status'

        else:
            rospy.logerr('Method getSrvPath received a bad request on type.')
            return -1

