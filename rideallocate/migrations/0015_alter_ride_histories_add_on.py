# Generated by Django 4.2.5 on 2024-09-25 14:57

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('rideallocate', '0014_shiftvehiclesdata'),
    ]

    operations = [
        migrations.AlterField(
            model_name='ride_histories',
            name='add_on',
            field=models.DateTimeField(auto_now_add=True, null=True),
        ),
    ]
