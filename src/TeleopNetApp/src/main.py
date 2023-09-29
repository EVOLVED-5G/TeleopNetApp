#!/usr/bin/env python3
from evolved5g import swagger_client
from evolved5g.swagger_client import LoginApi, User

from evolved5g.swagger_client.rest import ApiException

from evolved5g.sdk import QosAwareness
from evolved5g.swagger_client import UsageThreshold

from multiprocessing import Process
import rospy
import time
import json

import os

from std_msgs.msg import String

import evolvedApi.simulator as simulator


def create_guaranteed_bit_rate_subscription_teleoperation():

    netapp_id = os.environ.get('NETAPP_ID')
    host = simulator.get_host_of_the_nef_emulator()
    qos_awereness = QosAwareness(nef_url=host,
                                 folder_path_for_certificates_and_capif_api_key=simulator.get_folder_path_for_certificated_and_capif_api_key(),
                                 capif_host=simulator.get_capif_host(),
                                 capif_https_port=simulator.get_capif_https_port())

    equipment_network_identifier = os.environ.get('UE_ID')
    network_identifier = QosAwareness.NetworkIdentifier.IP_V4_ADDRESS
    conversational_voice = QosAwareness.GBRQosReference.CONVERSATIONAL_VOICE

    uplink = QosAwareness.QosMonitoringParameter.UPLINK

    uplink_threshold = 20
    gigabyte = 1024 * 1024 * 1024
    # Up to 10 gigabytes 5 GB downlink, 5gb uplink
    usage_threshold = UsageThreshold(duration= None, # not supported
                                     total_volume=10 * gigabyte,  # 10 Gigabytes of total volume
                                     downlink_volume=5 * gigabyte,  # 5 Gigabytes for downlink
                                     uplink_volume=5 * gigabyte  # 5 Gigabytes for uplink
                                     )

    notification_destination=os.environ.get('DESTINATION_ADDRESS')

    subscription = qos_awereness.create_guaranteed_bit_rate_subscription(
        netapp_id=netapp_id,
        equipment_network_identifier=equipment_network_identifier,
        network_identifier=network_identifier,
        notification_destination=notification_destination,
        gbr_qos_reference=conversational_voice,
        usage_threshold=usage_threshold,
        qos_monitoring_parameter=uplink,
        threshold=uplink_threshold,
        reporting_mode= QosAwareness.EventTriggeredReportingConfiguration(wait_time_in_seconds=10)
    )
    print("--- PRINTING THE SUBSCRIPTION WE JUST CREATED ----")
    print(subscription)

    # Request information about a subscription
    id = subscription.link.split("/")[-1]
    subscription_info = qos_awereness.get_subscription(netapp_id, id)
    print("--- RETRIEVING INFORMATION ABOUT SUBSCRIPTION " + id + "----")
    print(subscription_info)

def read_and_delete_all_existing_subscriptions():
    # How to get all subscriptions
    netapp_id = os.environ.get('NETAPP_ID')
    host = simulator.get_host_of_the_nef_emulator()
    qos_awareness = QosAwareness(nef_url=host,
                                 folder_path_for_certificates_and_capif_api_key=simulator.get_folder_path_for_certificated_and_capif_api_key(),
                                 capif_host=simulator.get_capif_host(),
                                 capif_https_port=simulator.get_capif_https_port())

    try:
        all_subscriptions = qos_awareness.get_all_subscriptions(netapp_id)
        print(all_subscriptions)

        for subscription in all_subscriptions:
            id = subscription.link.split("/")[-1]
            print("Deleting subscription with id: " + id)
            qos_awareness.delete_subscription(netapp_id, id)
    except ApiException as ex:
        if ex.status == 404:
            print("No active transcriptions found")
        else: #something else happened, re-throw the exception
            raise

def timer_qos(event):
    qos_msg = String()
    qos = simulator.read_qos()
    qos_msg.data = str(qos.json()['eventReports'][-1]['event'])
    pub.publish(qos_msg)


if __name__ == "__main__":
    create_guaranteed_bit_rate_subscription_teleoperation()
    rospy.sleep(15)
    pub = rospy.Publisher('qos', String, queue_size=10)
    rospy.init_node('qos_node', anonymous=True)
    rospy.Timer(rospy.Duration(0.5), timer_qos)
    rospy.spin()
    read_and_delete_all_existing_subscriptions()
    rospy.sleep(15)