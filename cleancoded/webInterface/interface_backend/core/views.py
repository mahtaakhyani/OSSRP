from pathlib import Path
import sys,os
import socket
import fcntl
import struct
import arpreq
# Create your views here."\move.py""\interface_backend\interface_backendapp\views.py"
from django.http import HttpRequest, HttpResponse, request, JsonResponse, StreamingHttpResponse, HttpRequest
from django.core.files.storage import Storage
from django.shortcuts import render
from django.views import View
from django.views.generic import TemplateView
from django.template.response import TemplateResponse
from rest_framework.request import Request
from rest_framework.views import APIView
from rest_framework.response import Response    
import subprocess


ws_dir = str(Path(__file__).resolve().parent.parent.parent)
sys.path.insert(0, ws_dir)

# import rospkg
# rospack = rospkg.RosPack()
# pkg = rospkg.RosPack().get_path('infrastructure')
# module_path = os.path.join(pkg, 'scripts', 'tools', 'helper_modules')
# sys.path.append(module_path)
# from django_ros_handler import ROSHandler as RH

# processes = os.popen('ps -ef').read().splitlines()

# # Iterate through the processes and kill the ones that match "ros"
# for process in processes:
#     if "ros" in process:
#         pid = process.split()[1]
#         os.system(f"kill {pid}")

# def check_rosmaster():
#     try:
#         subprocess.check_output(["rosnode", "list"])
#         return True
#     except subprocess.CalledProcessError:
#         return False
    
# if check_rosmaster():
#     import rospkg
#     rospack = rospkg.RosPack()
#     pkg = rospkg.RosPack().get_path('infrastructure')
#     module_path = os.path.join(pkg, 'scripts', 'tools', 'helper_modules')
#     sys.path.append(module_path)
#     from django_ros_handler import ROSHandler as RH
# else:
#     print("Master not running. starting master node and launch files...")
#     subprocess.call(["roslaunch", "infrastructure", "init_robot.launch"])
    
#     import rospkg
#     rospack = rospkg.RosPack()
#     pkg = rospkg.RosPack().get_path('infrastructure')
#     module_path = os.path.join(pkg, 'scripts', 'tools', 'helper_modules')
#     sys.path.append(module_path)
#     from django_ros_handler import ROSHandler as RH


from core.serializers import *
from core.models import *
from soundHandler.models import *
import serialHandler.serializers as serializers
from serialHandler.views import ParrotCommandController
from django.contrib.auth.decorators import login_required
from django.utils.decorators import method_decorator


@method_decorator(login_required, name='dispatch')
class MainViewTemp(APIView):
    def get(self, request):
        port = 8000
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            hostname = socket.inet_ntoa(fcntl.ioctl(
                sock.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack('256s', 'wlan0'[:15].encode('utf-8'))
            )[20:24])
        except IOError:
            hostname = 'hooshang-desktop.local'

        robot_name = request.build_absolute_uri().split('/')[-1]
        # print(voices_uri)
        emdb = EmotionModel.objects.all().order_by('-id')[0:]
        for emotion in emdb:
            # Split face_video_url on "/" 
            url_parts = emotion.face_video_url.split('/')
            # Replace first part with hostname if local
            if 'hoosh' in url_parts[2] or '192' in url_parts[2] or 'local' in url_parts[2]:
                try:
                    file_name = str(emotion.video_file.file).split('/')[-1]
                except:
                    file_name = url_parts[-1]
                new_url = f'http://{hostname}:{port}/api/stream?path=media/videos/{file_name}'
                # Update field
                emotion.face_video_url = new_url

                # Save
                emotion.save()
        sdb = Song.objects.all().order_by('-id')[0:]
        for song in sdb:
            # Split audio_link on "/" 
            url_parts = song.audio_link.split('/')  

            # Replace first part with hostname if local
            if 'hoosh' in url_parts[2] or '192' in url_parts[2] or 'local' in url_parts[2]:
                try:
                    file_name = str(song.audio_file.file).split('/')[-1]
                except:
                    file_name = url_parts[-1]
                new_url = f'http://{hostname}:{port}/api/stream?path=media/sounds/{file_name}'

                # Update field
                song.audio_link = new_url

                # Save
                emotion.save()
        # print(EmotionModel.objects.all()[1].sound.path())
        parrot_serializers_to_parse = ParrotCommandController().get(request)
        return TemplateResponse(request, 
            f'Modified_files/{robot_name}.html',
            {'emotions':emdb,
            'voices':sdb,
            'p_commands':parrot_serializers_to_parse
            }) #Sending the data to the template for rendering

#------------------------------- Emotion handling ----------------------------------------
class CoreReqHandler(APIView):

        default_exp = ''
        data = {'face' : '',
                        'face_url': '',	
                        'sound' : '',
                        'status' :500,
                        'message' : 'No data received'
                    } #Default data to be sent to the client if no data is received and prevent errors
            
        @classmethod
        def get(cls,request):
            serializer = EmotionModelSerializer(request)
            print(cls.default_exp)
            ''' 
            Get Param 'face' sent from the front-end (to the URL:  /reqpub)
            must contain the name of an emotion in the form of String data.
            e.g. "(i.e. /reqpub?face=normal or laugh,upset,surprise or shy)"
              '''
            if request.GET.get('face'):
                cls.default_exp = request.GET.get('face')
            else:
                pass
            cls.exp_fetched_db = serializer.data.keys()
            cls.sound_file = request.GET.get('sound')
            cls.face_video_url = request.GET.get('face_video_url')

            print(cls.exp_fetched_db,'cls.sound_file')
            cls.data = {'face' : cls.default_exp,
                        'face_url':cls.face_video_url,
                        'sound' : cls.sound_file,
                        'status' : HttpResponse.status_code,
                        'message' : 'The emotion has been set' 
                    } #Recieved Commands from the user
            return  JsonResponse(cls.data, status=200) #JSON response is also sent as the response on the URL: /reqpub to the user just in case! Feel free to ignore it.
        #     return cls.startstream()

        # @classmethod
        # def startstream(cls):
        #     cls.stream_var = requests.get(cls.default_exp,stream= True)
        #     return StreamingHttpResponse(streaming_content=cls.stream_var.raw)

        @classmethod
        def __str__(cls):
           return cls.data
        '''
        Returning the emotion name in the form of String
        to be fetched by the client (i.e. the front-end, android app, etc.)

        * Hint: Defined __str__ function is accessible from outside the class also. 
    '''

                            
# ----------------------------------------- Movement handling ----------------------------------

    # def post(self, request): #POST request must contain(def == default value): {pos_up(def=530),pos_right(def=500),right_hand(def=200),left_hand(def=874),speed(def=250),theta(def=0),yaw(def=0}

    #     self.requested_move = {'pos_up': request.POST['pos_up'],
    #                             'pos_right': request.POST['pos_right'],
    #                             'right_hand': request.POST['right_hand'],
    #                             'left_hand': request.POST['left_hand'],  
    #                             'speed': request.POST['speed'],  
    #                             'theta': request.POST['theta'],  
    #                             'yaw': request.POST['yaw'],  
    #                             }
    #     defaultvals = {'pos_up': '530', 'pos_right': '500',
    #                  'right_hand': '200', 'left_hand': '874',
    #                  'speed': '250' , 'theta': '0', 'yaw':'0'}

    #     self.command = [[k,v] for k,v in self.requested_move.items() if v not in defaultvals.values()]

''' Form must send POST data considering the instructions below:
            1) All sent data must be strictly within the ranges given in the instructions of models.py e.g. For the Hand-right-top --> {'right_hand': '2101220'}
                  (If not, substraction/sum must be included in move.py on each needed object, after recieving data as a list.)

            2) Any field if left empty, acts on it's default value as pre-defined specifically in models.py (Which obviously means you only need to send what you want to change.)
       '''
       
        # move.move(self.command) #Sending data to move.py move().

'''
        Notice: POST method is only used to make the API extendable for the future developments
          i.e. Implementing joystick or etc. Also it gives the advantage of being able to change the moving mechanisms
          without any need to change the code entirely.
          Using GET method is enough to make API usable for the current version but it is not encouraged as
          it is more efficient and precise-commanded to use POST method.

    '''

        
    #     return HttpResponse(
    #         json.dumps(self.requested_move),
    #         status=200, 
    #         content_type='application/json; charset=utf8')



class EmotionCommandController(APIView):

    def dummy(self): #If no emotion is selected (aka. the web interface hasn't loaded yet), the default emotion is set to 'neutral'
        default_exp = 'neutral'
        fetched_db = EmotionModel.objects.all()
        try:
            reqed_vidsrc_url = fetched_db.get(face=default_exp).face_video_url
            if reqed_vidsrc_url:
                try:
                    reqed_soundsrc_url = fetched_db.get(face=default_exp).sound.audio_link
                except:
                    reqed_soundsrc_url = "No assigned sound found"
        except:
            reqed_vidsrc_url = "No matching face found"
            reqed_soundsrc_url = ""

        data = {'face':reqed_vidsrc_url,
                    'sound' : reqed_soundsrc_url,
                    'status' : 200,
                    'message' : 'The emotion has been set' 
                    }
        return data

    def get(self, request): 
        self.api_response_data = CoreReqHandler.__str__()
        if self.api_response_data['face'] == '':
            data = self.dummy()
        else:
            data = {
                "face": self.api_response_data['face'],
                "sound": self.api_response_data['sound'],
                "status": self.api_response_data['status'],
                "message": self.api_response_data['message'],
            }
            '''
                (Passes the user-selected emotion's data to the robot)
                Taking in the latest user-commanded facial expression(emotion) through CoreReqHandler       and returning the corresponding String to the client in the form of JSON data.
                (Data is being sent on the URL:/reqcli)
                -Which here the client would be the android app sending requests to the server to
                update the face based on the new commands.
            '''
        print(data)
        return JsonResponse(data, status=201)

    def post(self, request):
        self.response_data = request.data #Recieving android-client response data on the URL:/reqcli
        self.response_data = {"Client response" : str(self.response_data)}
        print(self.response_data)
        return JsonResponse(self.response_data, status=201)


class EmotionModelViewDB(APIView):
    
    def get(self, request):
        fetched_db = EmotionModel.objects.all()
        reqed_emo_src = request.GET.get("face") #Recieving web-client response data on the URL:/reqcli
        try:
            reqed_vidsrc_url = fetched_db.get(face=reqed_emo_src).face_video_url
            if reqed_vidsrc_url:
                try:
                    reqed_soundsrc_url = fetched_db.get(face=reqed_emo_src).sound.audio_link
                except:
                    reqed_soundsrc_url = "No assigned sound found"
        except:
            reqed_vidsrc_url = "No matching face found"
            reqed_soundsrc_url = ""

        return JsonResponse( 
            data={
            "face_url": reqed_vidsrc_url,
            "sound_url": reqed_soundsrc_url,
         } ,status=201)

# ----------------------------------------- IP handling ----------------------------------
# Sending the local IP address of the Jetson Nano to the client (i.e. the web interface) on page load.
# This is done to make the server needless of any static IP address or dependent on the network router.
# NOTE: This is not a secure way of handling IP addresses and is only used for the sake of simplicity and ONLY for trusted networks.
# Using a DNS server is mandatory for this project to be deployed or used on a public network.
# -----------------------------------------------------------------------------------------
class IPUpdater(APIView):
    def get(self,request: HttpRequest):
        android_mac = "3c:f7:a4:50:a2:d1"
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            hostname = socket.inet_ntoa(fcntl.ioctl(
                sock.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack('256s', 'wlan0'[:15].encode('utf-8'))
            )[20:24])
        except IOError:
            hostname = 'hooshang-desktop.local'

        ip = ""
        gateway = f"{'.'.join(hostname.split('.')[:3])}"
        print(gateway)
        ip = self.find_ip_address(gateway,android_mac)
        print(f"Hostname: {hostname}")
        return JsonResponse(data={"host": hostname,
                                "android_ip": ip}, status=200)
   
  

    def find_ip_address(self,gateway, mac_address):
        for ip in range(1, 255):
            ip_address = f"{gateway}.{ip}"
            resolved_mac = arpreq.arpreq(ip_address)
            try:
                resolved_mac = arpreq.arpreq(ip_address)
                if resolved_mac.lower() == mac_address.lower():
                    return ip_address
            except :
                continue
        
        
        iprange =f'{gateway}.0/24'
        android_mac = mac_address.upper()
        out = self.run_nmap_with_password(android_mac,iprange)
        print(out)
        return out

    def run_nmap_with_password(self, mac_address, ip_range):
        print("MAC address of the android not found. Trying different method.")
        command = f"sudo nmap -sn -PR {ip_range}"
        process = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, error = process.communicate()
        output = output.decode()
        output = str(output).split("\n")
        print(str(output).split("\n"))
        for i,line in enumerate(output):
            if mac_address in line:
                ip = output[i-2].replace("Nmap scan report for ","")
                return ip
        
# ----------------------------------------- ROS handling ----------------------------------

class ProViewTemp(APIView):
    def get(self, request):
        try:
            rh = RH()
            img_topics_list = rh.get_image_topics()
            all_topics_list = rh.get_all_topics()
        except:
            img_topics_list = []
            all_topics_list = []
        emdb = EmotionModel.objects.all().order_by('-id')[0:]
        sdb = Song.objects.all().order_by('-id')[0:]
        return TemplateResponse(request, 
            f'Modified_files/pro.html',
            {'emotions':emdb,
            'voices':sdb,
            'img_ros_topics':img_topics_list,
            'topics': all_topics_list
            }) #Sending the data to the template for rendering


class GetMsgType(APIView):
    def get(self,request):
        req_topic = request.GET.get('topic')
        print(req_topic)
        try:
            msg_fields = RH().get_topic_type(req_topic)
            msg_type = RH().get_msg_type(req_topic)
            print(msg_type)
            return JsonResponse(data={"msg_type": str(msg_type)}, status=200)
        except:
            return JsonResponse(data={"msg_type": "Unable to communicate with master node."}, status=200)
        # return JsonResponse(data={"msg_type": "std_msg/String"}, status=200)

    
