# Generated by Django 3.2.9 on 2023-10-10 07:26

from django.db import migrations, models
import multiselectfield.db.fields


class Migration(migrations.Migration):

    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='JointConfig',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('name', models.CharField(default='My_Joint', max_length=255)),
            ],
        ),
        migrations.CreateModel(
            name='MotorConfig',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('type', multiselectfield.db.fields.MultiSelectField(choices=[('Type', 'Servo'), ('Type', 'DC'), ('Type', 'Stepper')], max_length=25)),
            ],
        ),
        migrations.CreateModel(
            name='MovingFeatures',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
            ],
        ),
        migrations.CreateModel(
            name='RobotConfig',
            fields=[
                ('id', models.AutoField(primary_key=True, serialize=False)),
                ('name', models.CharField(default='My_Robot', max_length=255)),
                ('streaminput', models.BooleanField(default=False)),
                ('saveinput', models.BooleanField(default=False)),
                ('voicefeatureextraction', models.BooleanField(default=False)),
                ('nlp', models.BooleanField(default=False)),
                ('tts', models.BooleanField(default=False)),
                ('emospeech', models.BooleanField(default=False)),
                ('emofacial', models.BooleanField(default=False)),
                ('move', models.BooleanField(default=False)),
                ('joints', models.ManyToManyField(to='setupWizard.JointConfig')),
            ],
        ),
        migrations.AddField(
            model_name='jointconfig',
            name='dofs',
            field=models.ManyToManyField(to='setupWizard.MotorConfig'),
        ),
    ]
