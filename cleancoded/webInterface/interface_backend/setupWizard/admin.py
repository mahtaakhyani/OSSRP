from django.contrib import admin
from setupWizard import models as mdls
# Register your models here.

admin.site.register(mdls.RobotConfig)
admin.site.register(mdls.MovingFeatures)
admin.site.register(mdls.JointConfig)
admin.site.register(mdls.MotorConfig)
