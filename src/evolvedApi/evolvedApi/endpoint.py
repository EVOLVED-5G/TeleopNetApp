from flask import Flask, render_template, request, jsonify, make_response
from flask_cors import CORS

app = Flask(__name__)
CORS(app)
app.config["DEBUG"] = True
qos = {'transaction': '', 
    'ipv4Addr': '', 'eventReports': 
    [{'event': 'QOS_GUARANTEED', 'accumulatedUsage': 
    {'duration': None, 'totalVolume': None, 'downlinkVolume': None, 'uplinkVolume': None},
     'appliedQosRef': None, 'qosMonReports': [{'ulDelays': [0], 'dlDelays': [0], 'rtDelays': [0]}]}]}


@app.route('/', methods=['GET'])
def index():
    return "evolved5G echo web-server started"

@app.route('/monitoring/callback', methods=['POST'])
def qos_reporter():
    print("New notification retrieved:")
    print(request.get_json())
    global qos
    qos = request.get_json()
    return request.get_json()

@app.route('/qos', methods=['GET'])
def get_qos():
    print("Get qos:")
    print(type(qos))
    print(qos)
    return qos

if __name__ == '__main__':
    print("initiating")
    app.run(host='0.0.0.0')