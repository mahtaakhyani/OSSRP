# Generated by Django 4.2.6 on 2023-11-03 20:32

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('soundHandler', '0002_alter_song_audio_link'),
    ]

    operations = [
        migrations.AlterField(
            model_name='song',
            name='audio_link',
            field=models.CharField(blank=True, default='http://<localhost_ip>:8000/api/stream?path=media/sounds/<your_audio_path>', max_length=200, null=True),
        ),
    ]
