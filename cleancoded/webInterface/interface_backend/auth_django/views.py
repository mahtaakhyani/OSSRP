from django.shortcuts import render
from django.http import  JsonResponse, HttpResponseRedirect
from django.template.response import TemplateResponse
from rest_framework.views import APIView
from setupWizard.models import RobotConfig
from core.views import MainViewTemp
from django.contrib.auth import update_session_auth_hash, login, logout, authenticate

# Create your views here.
class ChooseRobot(APIView):
 
    def get(self,request):
        db = RobotConfig.objects.all().order_by('-id')[0:]
        return TemplateResponse(request, 
            'Modified_files/selectrobot.html',
            {'robots':db})
    
    def post(self, request):
        msg = {
            'msg':'success'
        }
        return JsonResponse(msg, status=201)


class Login(APIView):
    def get(self,request):
        logout(request)
        return HttpResponseRedirect('/auth/choose')
    
    def post(self, request):
        username = request.POST["username"]
        password = request.POST["password"]
        url = request.POST["url"]
        user = authenticate(request, username=username, password=password)
        if user is not None:
            login(request, user)
            # Redirect to a success page.
            # redirect to url
            return HttpResponseRedirect(url)

        else:
            # Return an 'invalid login' error message.
            msg = {
            'msg':'Login Failed'
            }
            return JsonResponse(msg, status=500)


