from django.shortcuts import render
from django.urls import path , re_path
from setupWizard import views
from django.views.generic import TemplateView

urlpatterns = [
    re_path('setup', views.WizardWindow.as_view()),
 
]




