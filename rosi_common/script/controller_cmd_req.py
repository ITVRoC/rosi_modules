#!/usr/bin/env python3
'''this is the ROSI min controller main conscious. It receives Control commands to ROSI and filters it to send to 
rosi_controller. Its main need is to driblate the originId bullshit.

'''
import rospy
from controller.msg import Control
from controller.srv import RequestID

from rosi_common.rosi_tools import ctrlType

from rosi_common.node_status_tools import nodeStatus
from rosi_common.srv import SetNodeStatus, GetNodeStatusList

class NodeClass():

    def __init__(self, node_name):
        '''Class constructor'''
        self.node_name = node_name


        ##==== Useful variables
        # node status object
        self.ns = nodeStatus(node_name)

        self.lastMode = [ctrlType["Velocity"]]*8
        self.lastCmd = [0.0]*8
        
		# ROSI Controller requirements
        self.originID = 0

        # registering to publishers
        self.pub_rosi_cmd = rospy.Publisher('/rosi/rosi_controller/input', Control, queue_size=10)

        # registering to subscribers
        self.sub_flp_mind_req = rospy.Subscriber('/rosi/controller/req_cmd', Control, self.cllbck_rosi_mind_req)

        # registering for the requestID service
        rospy.loginfo('waiting until /rosi/rosi_controller/request_id service appears.')
        rospy.wait_for_service('/rosi/rosi_controller/request_id')
        self.srvcl_reqId = rospy.ServiceProxy('/rosi/rosi_controller/request_id', RequestID)
        self.requestId()

        # node management services
        srv_getStatus = rospy.Service(self.ns.getSrvPath('getNodeStatus', rospy), GetNodeStatusList, self.srvcllbck_getStatus)
        srv_setActive = rospy.Service(self.ns.getSrvPath('active', rospy), SetNodeStatus, self.srvcllbck_setActive)
        srv_setHaltCmd = rospy.Service(self.ns.getSrvPath('haltcmd', rospy), SetNodeStatus, self.srvcllbck_setHaltCmd)

        rospy.loginfo('node in spin mode, waiting for receiving messages')
        rospy.spin()


    # callback function
    def cllbck_rosi_mind_req(self, msg):

        if self.ns.getNodeStatus()['active']: # only runs if node is active

            # only publishes Velocity commands. If request in "unchanged", publishes last valid velocity command
            pub_cmd = []
            pub_modes = []
            for mode,cmd, modeLast, cmdLast in zip(msg.modes, msg.data, self.lastMode, self.lastCmd):
                if mode == ctrlType["Velocity"]:
                    pub_cmd.append(cmd)
                    pub_modes.append(mode)
                elif mode == ctrlType["Unchanged"]:
                    pub_cmd.append(cmdLast)
                    pub_modes.append(modeLast)


            # if node is commanded to send halt commands
            if self.ns.getNodeStatus()['haltcmd']:  
                pub_cmd = [0.0] * 8
                pub_modes = [ctrlType["Velocity"]] * 8

            # publishing the message
            pub_msg = Control()
            pub_msg.header.stamp = msg.header.stamp
            pub_msg.header.frame_id = "controller_cmd_req"
            pub_msg.originId = self.originID
            pub_msg.modes = pub_modes
            pub_msg.data = pub_cmd
            self.pub_rosi_cmd.publish(pub_msg)

            #print(pub_msg)

            # saving this command
            self.lastMode = pub_modes
            self.lastCmd = pub_cmd
        

    # requests id from ROSI controller
    def requestId(self):

        srv = RequestID()
        self.originID = self.srvcl_reqId().id

        rospy.loginfo("received originID: %d ", self.originID)


    ''' === Service Callbacks === '''
    def srvcllbck_getStatus(self, req):
        ''' Method for returning the node status flag list'''
        return self.ns.getNodeStatusSrvResponse()

    def srvcllbck_setActive(self, req):
        ''' Method for setting the active node status flag'''
        return self.ns.defActiveServiceReq(req, rospy)

    def srvcllbck_setHaltCmd(self, req):
        ''' Method for setting the haltCmd node status flag'''
        return self.ns.defHaltCmdServiceReq(req, rospy)


if __name__ == '__main__':
    node_name = 'controller_cmd_req'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('node '+node_name+' initiated.')
    try:
        node_obj = NodeClass(node_name)
    except rospy.ROSInternalException: pass



    