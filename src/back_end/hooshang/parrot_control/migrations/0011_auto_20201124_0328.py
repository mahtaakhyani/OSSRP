# Generated by Django 2.2.7 on 2020-11-24 03:28

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('parrot_control', '0010_parrotcommand_priority'),
    ]

    operations = [
        migrations.CreateModel(
            name='BlueParrotCommand',
            fields=[
            ],
            options={
                'proxy': True,
                'indexes': [],
                'constraints': [],
            },
            bases=('parrot_control.parrotcommand',),
        ),
        migrations.CreateModel(
            name='RedParrotCommand',
            fields=[
            ],
            options={
                'proxy': True,
                'indexes': [],
                'constraints': [],
            },
            bases=('parrot_control.parrotcommand',),
        ),
        migrations.AlterField(
            model_name='parrotcommand',
            name='voice_file_name',
            field=models.CharField(default='', max_length=512),
        ),
    ]
