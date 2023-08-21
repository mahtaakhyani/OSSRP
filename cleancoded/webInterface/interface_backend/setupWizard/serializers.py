from django.db.models import fields
from rest_framework import serializers
from setupWizard.models import *

class RobotModelSerializer(serializers.ModelSerializer):
    class Meta:
        model = RobotConfig
        fields = 'name','streaminput','saveinput','voicefeatureextraction','nlp','tts','emospeech','emofacial','move','joints'
        

    def create(self, validated_data):
        return RobotConfig.objects.create(**validated_data)

    def validate(self, attrs):
        return super().validate(attrs)