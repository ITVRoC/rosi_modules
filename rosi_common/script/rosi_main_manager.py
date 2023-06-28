#!/usr/bin/env python3
import rospy
import rosnode
from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import *
from sensor_msgs.msg import Joy


class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name

        ##==== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        # checking if kinova gen3 nodes are running
        nodes_list = rosnode.get_node_names()
        if '/my_gen3_driver' in nodes_list:
            self.flag_gen3_nodesEnabled = True
        else:
            self.flag_gen3_nodesEnabled = False


        ##=== PARAMETERS
        self.ls_nodes_managed = (  # nodes names that are managed by this manager node
                'joy_base_cmdVel',          
                'joy_gen3',                  
                'joy_flippers_cmdVel',  
                'flippers_safety', 
                #'flippers_cmd_sum',  
                'traction_cmd_sum',
                'chassis_control' 
        )

        self.d_operative_mode = {
            'base_flp_jnt_spc': 1,
            'base_flp_base_spc': 2,
            'base_posture_cntrl': 3,
            'gen3': 4
        }


        ##=== Managed nodes pre-defined state machine
        # Boolean lists are the respective desired state of each managed node (following the list order in self.ls_nodes_managed) for each rosi main machine state
        
        self.nodes_ON_initial_enabled = [True, False, True, False, True, False]
        self.nodes_ON_initial_halt =  [False, False, False, False, False, False]

        self.nodes_OFF_enabled = [False, False, False, False, False, False]
        self.nodes_OFF_halt = [False, False, False, False, False, False]
        

        if self.flag_gen3_nodesEnabled: # when gen3 is up and running
            self.d_operative_modes_enabled = {
                'base_flp_jnt_spc': [True, True, True, False, True, False],
                'base_flp_base_spc': [True, True, False, True, True, False],
                'base_posture_cntrl': [True, True, False, True, True, True],
                'gen3': [True, True, True, False, True, False]
            }
            self.d_operative_modes_halt = {
                'base_flp_jnt_spc': [False, True, False, False, False, False],
                'base_flp_base_spc': [False, True, False, False, False, False],
                'base_posture_cntrl': [False, True, False, False, False, False],
                'gen3': [True, False, True, False, False, False]
            }
        else: # when gen3 is not installed
            self.d_operative_modes_enabled = {
                'base_flp_jnt_spc': [True, False, True, False, True, False],
                'base_flp_base_spc': [True, False, False, True, True, False],
                'base_posture_cntrl': [True, False, False, True, True, True],
                'gen3': [False, False, False, False, False, False]
            }
            self.d_operative_modes_halt = {
                'base_flp_jnt_spc': [False, False, False, False, False, False],
                'base_flp_base_spc': [False, False, False, False, False, False],
                'base_posture_cntrl': [False, False, False, False, False, False],
                'gen3': [False, False, False, False, False, False]
            }


        ##=== USEFUL VARIABLES
        self.operation_onOff_current = False # system is initially turned off
        self.operative_mode_current = self.d_operative_mode['base_flp_jnt_spc'] # rosi initial operative state is in flipppers joint state


        ##=== MANAGED NODES SERVICES INITIALIZATION
        # retrieving nodes services path
        self.nsNodes = ['/node_status/'+node_name for node_name in self.ls_nodes_managed]

        # retrieving getNodeStatus service handlers
        self.l_sh_getNodesStatus = [self.getServiceHandle(ns+'/get_node_status', GetNodeStatusList) for ns in self.nsNodes]

        # retrieving set Active service handlers
        self.l_sh_setEnabled = [self.getServiceHandle(ns+'/def_active', SetNodeStatus) for ns in self.nsNodes]

        # retrieving set Halt service handlers
        self.l_sh_setHalt = [self.getServiceHandle(ns+'/def_haltcmd', SetNodeStatus) for ns in self.nsNodes]


        ###=== INITIAL ZERO STATE
        # setting state zero: all nodes disabled
        rospy.loginfo('[manager] Setting all nodes to the default OFF state.')
        self.setNodesStateBulk(self.l_sh_setEnabled, self.nodes_OFF_enabled)
        self.setNodesStateBulk(self.l_sh_setHalt, self.nodes_OFF_halt)           

        ###=== ROS INTERFACES
        # services for modes
        srv_getOnOff = rospy.Service('/rosi/manager/get_on_off', GetNodeStatus, self.srvcllbck_getOnOff)
        srv_setOnOff = rospy.Service('/rosi/manager/set_on_off', SetNodeStatus, self.srvcllbck_setOnOff)
        srv_getOpState = rospy.Service('/rosi/manager/get_operational_state', GetInt, self.srvcllbck_getOpState)
        srv_setOpState = rospy.Service('/rosi/manager/set_operational_state', SetNodeIntState, self.srvcllbck_setOpState)
        

        ###=== SPIN MODE
        rospy.loginfo('[manager] Node in spin mode.')
        rospy.loginfo('[manager] ROSI IS IDLE AND READY TO OPERATE.')
        rospy.loginfo('[manager] Press Start + Select to enable operation.')
        rospy.spin()


    ###=== SERVICES

    def srvcllbck_getOnOff(self, req):
        '''Service callback to get ROSI ON OFF state'''
        return self.operation_onOff_current


    def srvcllbck_setOnOff(self, req):
        '''Service callback to define ROSI ON OFF state'''
        if req.set_value == True:
            rospy.loginfo('[manager] Setting nodes to default ON state.')
            self.setNodesStateBulk(self.l_sh_setEnabled, self.nodes_ON_initial_enabled)
            self.setNodesStateBulk(self.l_sh_setHalt, self.nodes_ON_initial_halt)
            self.operation_onOff_current = True
            aux_currentMode = [k for k, v in self.d_operative_mode.items() if v == req.set_value][0]
            rospy.loginfo('[manager] Current operational state is '+aux_currentMode)
        elif req.set_value == False:
            rospy.loginfo('[manager] Setting nodes to default OFF state.')
            self.setNodesStateBulk(self.l_sh_setEnabled, self.nodes_OFF_enabled)
            self.setNodesStateBulk(self.l_sh_setHalt, self.nodes_OFF_halt)
            self.operation_onOff_current = False
        else:
            rospy.logerr('Service setOnOff received a non-recognizable command.')

        return SetNodeStatusResponse(self.operation_onOff_current)


    def srvcllbck_getOpState(self, req):
        '''Service callback that returns current operative state'''
        return GetIntResponse(self.operative_mode_current)
    

    def srvcllbck_setOpState(self, req):
        '''Service callback that sets the operative state'''
        
        if req.set_value != self.operative_mode_current:
            desired_mode = [k for k, v in self.d_operative_mode.items() if v == req.set_value][0]

            # checking if the requested mode is gen3 but it is not installed.
            if desired_mode == 'gen3' and not self.flag_gen3_nodesEnabled:
                rospy.logwarn('Gen3 nodes are not running. Not possible to enter in this mode. Request not implemented.')
                return SetNodeIntStateResponse(-1)

            rospy.loginfo('[manager] Setting Operational state to '+desired_mode)
            self.setNodesStateBulk(self.l_sh_setEnabled, self.d_operative_modes_enabled[desired_mode])
            self.setNodesStateBulk(self.l_sh_setHalt, self.d_operative_modes_halt[desired_mode])
            self.operative_mode_current = self.d_operative_mode[desired_mode]
            return SetNodeIntStateResponse(self.operative_mode_current)
        
        elif req.set_value == self.operative_mode_current: # in case of requested mode is already running
            rospy.logwarn('Requested operative state already running. Request not implemented.')
            return SetNodeIntStateResponse(-1)
        
        elif req.set_value < 1 or req.set_value > 4:    # in case of requested mode does not exist
            rospy.logwarn('Requested operative state does not exist. Request not implemented.')
            return SetNodeIntStateResponse(-1)
        
        else:
            rospy.logerr('Unknown error with the requested operative state')
            return SetNodeIntStateResponse(-1)

    ###=== USEFUL METHODS

    @staticmethod
    def getParamWithWait(path_param):
        """Waits until a param exists so retrieves it"""
        while not  rospy.has_param(path_param):
            rospy.loginfo("[manager] Waiting for param: %s", path_param)
        return rospy.get_param(path_param)


    @staticmethod
    def getServiceHandle(service_path, service_type):
        '''Retrieves a service proxi after confirming it exists
        Input:
            - h_node_status: handler to the service that retrieves the node status'''
        rospy.loginfo("[manager] Waiting for service: %s", service_path)
        rospy.wait_for_service(service_path)
        rospy.loginfo("[manager] Service %s found!", service_path)
        rospy.loginfo("")
        return rospy.ServiceProxy(service_path, service_type)


    @staticmethod
    def getNodeEnabledState(h_node_status):
        '''Returns solely the node Enabled status
        Input:
            - h_node_status: handler to the service that retrieves the node status'''
        return h_node_status().node_status[0]
    

    @staticmethod
    def getNodeHaltState(h_node_status):
        '''Returns solely the node Halt status
        Input:
            - h_node_status: handler to the service that retrieves the node status'''
        return h_node_status().node_status[2]
    

    @staticmethod
    def setNodesStateBulk(l_sh_setState, l_state_desired):
        ''' Defines nodes state in a bulk
        '''
        for sh_setState, state in zip(l_sh_setState, l_state_desired):
                while not sh_setState(state):
                    rospy.loginfo('[manager] Enforcing node to enable.')
    

    def checkManagedNodesState(self,l_h_node_status, state_type, state_sum):
        '''Check if a list of managed nodes sums to a desired quantity
        Input
            - l_h_node_status: list containing handlers to all node status services.
            - state_type: type of state to check: 'enabled' or 'halt' 
            - state_sum: desired sum (for instance, sum zero means all nodes are disabled)
        '''
        aux = 0
        if state_type == 'enabled':
            for i in range(len(l_h_node_status)):
                aux+=self.getNodeEnabledState(l_h_node_status[i])
        elif state_type == 'halt':
            for i in range(len(l_h_node_status)):
                aux+=self.getNodeHaltState(l_h_node_status[i])
        else:
            rospy.logerr('node status type to check not recognized.')
            return -1
        
        if aux==state_sum:
            return True
        else:
            return False

    
if __name__ == '__main__':
    node_name = 'rosi_main_manager'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass