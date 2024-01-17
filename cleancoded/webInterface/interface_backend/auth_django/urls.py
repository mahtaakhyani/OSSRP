from django.shortcuts import render
from django.urls import path , re_path
from auth_django import views
from django.views.generic import TemplateView
from django.urls.conf import include

urlpatterns = [
    re_path('login', views.Login.as_view()),
    re_path('choose', views.ChooseRobot.as_view()),
 
]




