# Generated by Django 4.2.5 on 2024-09-24 05:55

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('rideallocate', '0003_caballocation_escortmanagement_histories_and_more'),
    ]

    operations = [
        migrations.CreateModel(
            name='VehiclesData',
            fields=[
                ('VehicleId', models.IntegerField(primary_key=True, serialize=False)),
                ('VehicleName', models.CharField(max_length=50)),
                ('VehicleNumber', models.CharField(max_length=100)),
                ('Mileage', models.FloatField()),
                ('YearOfManufacturing', models.CharField(max_length=4)),
                ('SeatCapacity', models.IntegerField()),
                ('VehicleType', models.CharField(max_length=50)),
                ('VehicleImage', models.CharField(max_length=300, null=True)),
                ('InsuranceNumber', models.CharField(max_length=100)),
                ('FuelType', models.CharField(max_length=30)),
                ('VehicleStatus', models.BooleanField()),
                ('VendorId', models.CharField(max_length=30)),
                ('VendorName', models.CharField(max_length=50)),
                ('DriverId', models.CharField(blank=True, max_length=255, null=True)),
                ('AddedDate', models.DateField(null=True)),
            ],
        ),
        migrations.DeleteModel(
            name='VehicleDetail',
        ),
        migrations.DeleteModel(
            name='Vehicles',
        ),
        migrations.RenameField(
            model_name='caballocation',
            old_name='seat_capacity',
            new_name='SeatCapacity',
        ),
        migrations.RenameField(
            model_name='caballocation',
            old_name='vehicle_number',
            new_name='VehicleNumber',
        ),
        migrations.RenameField(
            model_name='caballocation',
            old_name='Priority_Order',
            new_name='priority_order',
        ),
        migrations.RenameField(
            model_name='histories',
            old_name='seat_capacity',
            new_name='SeatCapacity',
        ),
        migrations.RenameField(
            model_name='histories',
            old_name='vehicle_number',
            new_name='VehicleNumber',
        ),
        migrations.RenameField(
            model_name='histories',
            old_name='Priority_Order',
            new_name='priority_order',
        ),
        migrations.RenameField(
            model_name='result',
            old_name='seat_capacity',
            new_name='SeatCapacity',
        ),
        migrations.RenameField(
            model_name='result',
            old_name='vehicle_number',
            new_name='VehicleNumber',
        ),
        migrations.RenameField(
            model_name='resultescortassign',
            old_name='seat_capacity',
            new_name='SeatCapacity',
        ),
        migrations.RenameField(
            model_name='resultescortassign',
            old_name='vehicle_number',
            new_name='VehicleNumber',
        ),
        migrations.RenameField(
            model_name='resultescortassign',
            old_name='Priority_Order',
            new_name='priority_order',
        ),
        migrations.RenameField(
            model_name='ride_histories',
            old_name='seat_capacity',
            new_name='SeatCapacity',
        ),
        migrations.RenameField(
            model_name='ride_histories',
            old_name='vehicle_number',
            new_name='VehicleNumber',
        ),
    ]
