from django.db.models import fields
from rest_framework import serializers
from hooshangapp import models


# Hooshang dynamixels-grouped movement(Head - hands - transition) serilizing
# class HooshangDynaSerializerHead(serializers.ModelSerializer):

#     class Meta:
#         model = models.EmotionModel
#         fields = ('id','pos_up','pos_right')

# class HooshangDynaSerializerHands(serializers.ModelSerializer):

#     class Meta:
#         model = models.EmotionModel
#         fields = ('id','right_hand','left_hand')

# class HooshangDynaSerializer(serializers.ModelSerializer):

#     class Meta:
#         model = models.EmotionModel
#         fields = ('id','speed','theta','yaw')

class EmotionModelSerializer(serializers.ModelSerializer):

    class Meta:
        model = models.EmotionModel
        fields = "__all__" #('id','face','sound')	
