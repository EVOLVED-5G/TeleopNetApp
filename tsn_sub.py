# TSN script to suscribe to the TSN connection

from evolved5g.sdk import TSNManager
import evolvedApi.simulator as simulator
import os

tsn_host = os.environ.get('TSN_HOST') # TSN server hostname
tsn_port = os.environ.get('TSN_PORT')  # TSN server port

netapp_name = "TeleopNetwokApp"  # The name of our NetApp

tsn = TSNManager(  # Initialization of the TNSManager
    folder_path_for_certificates_and_capif_api_key=simulator.get_folder_path_for_certificated_and_capif_api_key(),
    capif_host=simulator.get_capif_host(),
    capif_https_port=simulator.get_capif_https_port(),
    https=False,
    tsn_host=tsn_host,
    tsn_port=tsn_port
)

"""
Tpply a TSN profile configuration to the Teleop NetApp
"""
profiles = tsn.get_tsn_profiles()
profile_to_apply = profiles[-1]
profile_configuration = profile_to_apply.get_configuration_for_tsn_profile()
tsn_netapp_identifier = tsn.TSNNetappIdentifier(netapp_name=netapp_name)
clearance_token = tsn.apply_tsn_profile_to_netapp(
    profile=profile_to_apply, tsn_netapp_identifier=tsn_netapp_identifier
)
