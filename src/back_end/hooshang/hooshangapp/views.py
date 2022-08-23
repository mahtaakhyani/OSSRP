import json
from pathlib import Path
import sys
import requests
# Create your views here."\move.py""\hooshang\hooshangapp\views.py"
from django.http import HttpResponse, request, JsonResponse, StreamingHttpResponse
from django.core.files.storage import Storage
from django.shortcuts import render
from django.views import View
from django.views.generic import TemplateView
from django.template.response import TemplateResponse
from rest_framework.request import Request
from rest_framework.views import APIView
from rest_framework.response import Response    

ws_dir = str(Path(__file__).resolve().parent.parent.parent)
sys.path.insert(0, ws_dir)

from hooshangapp.serializers import *
from hooshangapp.models import *
from soundsapp.models import *


class MainViewTemp(APIView):
        def get(self, request):
            db = EmotionModel.objects.all().order_by('-id')[1:]
            # print(EmotionModel.objects.all()[1].sound.path())
            return TemplateResponse(request, 'Modified_files/Page-1.html', {'emotions':db})

#------------------------------- Emotion handling --------------------------
class CoreReqHandler(APIView):

        default_exp = ''
            
        @classmethod
        def get(cls,request):
            serializer = EmotionModelSerializer(request)
            print(cls.default_exp)
            ''' 
            Get Param 'face' sent from the front-end (to the URL:  /reqpub)
            must contain the name of an emotion in the form of String data.
            e.g. "(i.e. /reqpub?face=normal or laugh,upset,surprise or shy)"
              '''
            cls.default_exp = request.GET.get('face')
            cls.exp_fetched_db = serializer.data.keys()
            cls.sound_file = request.GET.get('sound')

            print(cls.exp_fetched_db,'cls.sound_file')
            cls.data = {'face' : cls.default_exp, 
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


    def get(self, request): 
        self.api_response_data = CoreReqHandler.__str__()
        data = {
            "face": self.api_response_data['face'],
            "sound": self.api_response_data['sound'],
            "status": self.api_response_data['status']
        }
        # print(self.requested_expression)
        '''
            Taking in the latest user-commanded facial expression(emotion) through CoreReqHandler       and returning the corresponding String to the client in the form of JSON data.
            (Data is being sent on the URL:/reqcli)
            -Which here the client would be the android app sending requests to the server to
            update the face based on the new commands.
        '''
        return JsonResponse(data, status=201)

    def post(self, request):
        self.response_data = request.data #Recieving client response data on the URL:/reqcli
        self.response_data = {"Client response" : str(self.response_data)}
        print(self.response_data)
        return JsonResponse(self.response_data, status=201)

   