# Generated by Django 4.2.5 on 2024-09-24 09:44

from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        ('rideallocate', '0008_alter_escortmanagement_escortaddeddate'),
    ]

    operations = [
        migrations.RenameModel(
            old_name='ResultEscortAssign',
            new_name='DropingData',
        ),
        migrations.RenameModel(
            old_name='Result',
            new_name='PickUpData',
        ),
    ]
