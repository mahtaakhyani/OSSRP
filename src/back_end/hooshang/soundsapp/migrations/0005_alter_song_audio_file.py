# Generated by Django 3.2.12 on 2022-08-29 20:44

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('soundsapp', '0004_auto_20220821_1301'),
    ]

    operations = [
        migrations.AlterField(
            model_name='song',
            name='audio_file',
            field=models.FileField(blank=True, null=True, upload_to='sounds/'),
        ),
    ]
