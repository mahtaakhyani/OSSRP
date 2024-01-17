from django.shortcuts import render
from django.urls import path , re_path
from core import views
from django.views.generic import TemplateView
from setupWizard.models import RobotConfig

urlpatterns = [
    re_path('reqpub', views.CoreReqHandler.as_view()), # The URL to push emotion commands on
    re_path('reqcli', views.EmotionCommandController.as_view()), # The URL to get latest emotion commands from
    re_path('reqemo', views.EmotionModelViewDB.as_view()), # The URL to get requested emotion datas from
    re_path('reqip', views.IPUpdater.as_view()), # The URL to fetch server local ip address
    re_path('get_msg_type', views.GetMsgType.as_view()), 
    re_path('index', views.MainViewTemp.as_view()), # The URL to load the index.html page
    re_path('pro', views.ProViewTemp.as_view()), 

    
]

db = RobotConfig.objects.all().order_by('-id')[0:]

for robot in db:
    urlpatterns += [
            re_path(f'{robot.name}', views.MainViewTemp.as_view()) 
            
    ]


