from evolved5g import swagger_client
from evolved5g.swagger_client import LoginApi, User
from evolved5g.swagger_client.models import Token
import requests, json

with open('config.json', 'r') as file:     config = json.load(file)

def get_token() -> Token:
    username = config['USERNAME']
    password = config['PASSWORD']
    # User name and pass matches are set in the .env of the docker of NEF_EMULATOR. See
    # https://github.com/EVOLVED-5G/NEF_emulator
    configuration = swagger_client.Configuration()
    # The host of the 5G API (emulator)
    configuration.host = get_host_of_the_nef_emulator()
    api_client = swagger_client.ApiClient(configuration=configuration)
    api_client.select_header_content_type(["application/x-www-form-urlencoded"])
    api = LoginApi(api_client)
    token = api.login_access_token_api_v1_login_access_token_post("", username, password, "", "", "")
    return token


def get_api_client(token) -> swagger_client.ApiClient:
    configuration = swagger_client.Configuration()
    configuration.host = get_host_of_the_nef_emulator()
    configuration.access_token = token.access_token
    api_client = swagger_client.ApiClient(configuration=configuration)
    return api_client

def read_qos() -> int:
    response = requests.get(url=config['ENDPOINT_ADDRESS'],
                            headers=None,
                            data=None
                            )
    return response

def get_host_of_the_nef_emulator() -> str:
    return config['HOST_OF_THE_NEF_EMULATOR']
