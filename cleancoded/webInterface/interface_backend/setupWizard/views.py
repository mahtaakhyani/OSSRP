from typing import Any
from django.shortcuts import render
from django.http import  JsonResponse
from django.template.response import TemplateResponse
from rest_framework.views import APIView
from setupWizard.serializers import RobotModelSerializer
# Create your views here.

class WizardWindow(APIView):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

    def get(self,request):
        return TemplateResponse(request, 
            'Modified_files/setupwizardrobot.html')
    
    def post(self, request):
        serializer = RobotModelSerializer()
        print(request.data.get('data'))
        try:
            serializer.create(request.data.get('data'))
        except Exception as e:
            print(e)

        msg = {
            'msg':'success'
        }
        return JsonResponse(msg, status=201)
