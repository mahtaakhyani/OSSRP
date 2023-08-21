from django.db import models
from multiselectfield import MultiSelectField


# Create your models here.
class MotorConfig(models.Model):
    choices_tuple = (('Type','Servo'),('Type','DC'), ('Type','Stepper'))
    type = MultiSelectField(max_length=25,max_choices=3,choices=choices_tuple)


class JointConfig(models.Model):
    name = models.CharField(max_length=255, default="My_Joint" )
    dofs = models.ManyToManyField(MotorConfig)


class MovingFeatures(models.Model):
    pass


class RobotConfig(models.Model):
    id = models.AutoField(primary_key=True)
    name = models.CharField(max_length=255, default="My_Robot" )

    streaminput = models.BooleanField(default=False)
    saveinput = models.BooleanField(default=False)
    voicefeatureextraction = models.BooleanField(default=False)
    nlp = models.BooleanField(default=False)
    tts = models.BooleanField(default=False)
    emospeech = models.BooleanField(default=False)
    emofacial = models.BooleanField(default=False)
    move = models.BooleanField(default=False)

    joints = models.ManyToManyField(JointConfig)

