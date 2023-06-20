from evolved5g import swagger_client
from evolved5g.swagger_client import LoginApi, User
import requests, json

import os

def read_qos() -> int:
    response = requests.get(url=os.environ.get('ENDPOINT_ADDRESS'),
                            headers=None,
                            data=None
                            )
    return response

def get_host_of_the_nef_emulator() -> str:
    return os.environ.get('HOST_OF_THE_NEF_EMULATOR')

def get_folder_path_for_certificated_and_capif_api_key()->str:
    """
    This is the folder that was provided when you registered the NetApp to CAPIF.
    It contains the certificates and the api.key needed to communicate with the CAPIF server
    :return:
    """
    return "/app/capif_onboarding"

def get_capif_host()->str:
    """
    When running CAPIF via docker (by running ./run.sh) you should have at your /etc/hosts the following record
    127.0.0.1       capifcore
    :return:
    """
    return os.environ.get('CAPIF_HOSTNAME')

def get_capif_https_port()->int:
    """
    This is the default https port when running CAPIF via docker
    :return:
    """
    return os.environ.get('CAPIF_PORT_HTTPS')