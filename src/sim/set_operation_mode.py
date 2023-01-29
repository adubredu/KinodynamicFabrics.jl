from ws4py.client.threadedclient import WebSocketClient
import json 
import sys
import time

class BasicClient(WebSocketClient):
    def opened(self):
        self.operation_mode = None
        self.responded = True
        # self.arm_pose = None

        privilege_request = ['request-privilege', 
                                {'privilege' : 'change-action-command',
                                 'priority' : 0}]
        self.send(json.dumps(privilege_request))


    def closed(self, code, reason):
        print(("Closed", code, reason))


    def received_message(self, m):
        dataloaded = json.loads(m.data)
        message_type = str(dataloaded[0])
        message_dict = dataloaded[1]

        if message_type == 'privileges':
            self.done = message_dict['privileges'][0]['has']
            if self.done:
                print("Privilege request executed successfully.")

        elif message_type == 'robot-status':
            self.responded = True
            self.operation_mode = str(message_dict['operation-mode'])

        elif message_type == 'error':
            self.error_info = str(message_dict['info'])
            print('Error: ', self.error_info)

        elif message_type == 'action-status-changed':
            if message_dict['status'] == 'running':
                # print('Command received and is running')
                self.completed = False

            elif message_dict['status'] == 'success':
                print('Command finished successfully executing: ', str(message_dict['info']))
                self.completed = True
            elif message_dict['status'] == 'error':
                print('Error: ', str(message_dict['info']))
                self.completed = True 

def set_operation_mode(mode, ws):
    msg = [
            "action-set-operation-mode",
            {
                "mode": mode
            }
        ]
    ws.send(json.dumps(msg))


if __name__ == "__main__":
    system = str(sys.argv[1])
    mode = str(sys.argv[2])
    if (system == "real"):
        ip = "ws://10.10.1.1:8080"
    else:
        ip = "ws://127.0.0.1:8080"

    ws = BasicClient(ip, protocols=["json-v1-agility"])
    ws.daemon = False 

    while True:
        try:
            ws.connect()
            print("WS connection established")
            time.sleep(1)
            break
        except:
            print('WS connection NOT established')
            time.sleep(1)

    set_operation_mode(mode, ws)
    time.sleep(5)
    ws.close()


    