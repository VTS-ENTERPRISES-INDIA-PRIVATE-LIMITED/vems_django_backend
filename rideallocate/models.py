from django.db import models
from django.utils import timezone


class CabBooking(models.Model):
    BookingId = models.CharField(max_length=100)
    Latitude = models.FloatField()
    Longitude = models.FloatField()
    EmployeeId = models.CharField(max_length=100)
    TripDate = models.DateField(null=True)
    LoginTime = models.CharField(max_length=100)
    LogoutTime = models.CharField(max_length=100)
    EmployeeName = models.CharField(max_length=200)
    EmployeeGender = models.CharField(max_length=10)
    EmployeeAddress = models.TextField(null=True)
    EmployeeCity = models.CharField(max_length=100)
    BookingDateTime = models.CharField(max_length=255)

class Ride_histories(models.Model):
    BookingId = models.CharField(max_length=100)
    EmployeeId = models.CharField(max_length=100)
    TripDate = models.DateField(null=True)
    LoginTime = models.CharField(max_length=100)
    LogoutTime = models.CharField(max_length=100)
    EmployeeName = models.CharField(max_length=200)
    EmployeeGender = models.CharField(max_length=10)
    EmployeeAddress = models.TextField()
    EmployeeCity = models.CharField(max_length=100)
    Latitude = models.FloatField()
    Longitude = models.FloatField()
    VehicleNumber = models.CharField(max_length=100, null=True)
    SeatCapacity = models.IntegerField(null=True)
    VehicleId = models.CharField(max_length=100,null=True)
    CumulativeTravelTime = models.FloatField(null=True)
    AddOn = models.DateTimeField(auto_now_add=True,null=True)
    PriorityOrder = models.IntegerField(null=True)

class Histories(models.Model):
    BookingId = models.CharField(max_length=100)
    EmployeeId = models.CharField(max_length=100)
    TripDate = models.DateField(null=True)
    LoginTime = models.CharField(max_length=100)
    LogoutTime = models.CharField(max_length=100)
    EmployeeName = models.CharField(max_length=255)
    EmployeeGender = models.CharField(max_length=10)
    EmployeeAddress = models.TextField()
    EmployeeCity = models.CharField(max_length=255)
    Latitude = models.FloatField()
    Longitude = models.FloatField()
    VehicleNumber = models.CharField(max_length=100, null=True)
    SeatCapacity = models.IntegerField(null=True)
    VehicleId = models.CharField(max_length=100,null=True)
    CumulativeTravelTime = models.FloatField(null=True)
    PriorityOrder = models.IntegerField(null=True, blank=True)
    EscortId = models.CharField(max_length=100,null=True)
    AddOn = models.DateTimeField(auto_now=True)

    
#Escorts tables

class VehiclesData(models.Model):    
    VehicleId = models.CharField(max_length=100,primary_key=True) 
    VehicleName = models.CharField(max_length=50,null=True)  
    VehicleNumber = models.CharField(max_length=100)  
    VehicleMileageRange = models.FloatField(null=True)  
    VehicleManufacturedYear = models.CharField(max_length=5,null=True) 
    VehicleSeatCapacity = models.IntegerField() 
    VehicleType = models.CharField(max_length=50,null=True) 
    VehicleImage = models.CharField(max_length=300,null=True) 
    InsuranceNumber = models.CharField(max_length=100,null=True)  
    FuelType = models.CharField(max_length=30,null=True)  
    VehicleStatus = models.BooleanField(null=True)
    VendorId = models.CharField(max_length=30,null=True)  
    VendorName = models.CharField(max_length=50,null=True)
    VehicleAddedDate = models.DateField(null=True)
    VehicleShift = models.CharField(max_length=100,null=True)

class ShiftVehiclesData(models.Model):    
    VehicleId = models.CharField(max_length=100,primary_key=True,null=False) 
    VehicleName = models.CharField(max_length=50,null=True)  
    VehicleNumber = models.CharField(max_length=100)  
    VehicleMileageRange = models.FloatField(null=True)  
    VehicleManufacturedYear = models.CharField(max_length=5,null=True) 
    VehicleSeatCapacity = models.IntegerField() 
    VehicleType = models.CharField(max_length=50,null=True) 
    VehicleImage = models.CharField(max_length=300,null=True) 
    VehicleInsuranceNumber = models.CharField(max_length=100,null=True)  
    VehicleFuelType = models.CharField(max_length=30,null=True)  
    VehicleStatus = models.BooleanField(null=True) 
    VendorId = models.CharField(max_length=30,null=True)  
    VendorName = models.CharField(max_length=50,null=True)
    VehicleAddedDate = models.DateField(null=True)
    VehicleShift = models.CharField(max_length=100,null=True)

class CabVacantDetails(models.Model):
    VehicleId = models.CharField(max_length=100,primary_key=True,null=False) 
    VehicleName = models.CharField(max_length=50,null=True)  
    VehicleNumber = models.CharField(max_length=100,null=True)  
    VehicleMileageRange = models.FloatField(null=True)  
    VehicleManufacturedYear = models.CharField(max_length=5,null=True) 
    VehicleSeatCapacity = models.IntegerField(null=True) 
    VehicleType = models.CharField(max_length=50,null=True) 
    VehicleImage = models.CharField(max_length=300,null=True) 
    VehicleInsuranceNumber = models.CharField(max_length=100,null=True)  
    VehicleFuelType = models.CharField(max_length=30,null=True)
    VehicleStatus = models.BooleanField(null=True) 
    VendorId = models.CharField(max_length=30,null=True)  
    VendorName = models.CharField(max_length=50,null=True)
    VehicleAddedDate = models.DateField(null=True)

class CabAllocation(models.Model):
    BookingId = models.CharField(max_length=100, unique=True)
    EmployeeId = models.CharField(max_length=100)
    TripDate = models.DateField(null=True)
    LoginTime = models.CharField(max_length=100)
    LogoutTime = models.CharField(max_length=100)
    EmployeeName = models.CharField(max_length=200)
    EmployeeGender = models.CharField(max_length=10)
    EmployeeAddress = models.TextField()
    EmployeeCity = models.CharField(max_length=100)
    Latitude = models.FloatField()
    Longitude = models.FloatField()
    VehicleNumber = models.CharField(max_length=100, null=True)
    SeatCapacity = models.IntegerField(null=True)
    VehicleId = models.CharField(max_length=100,null=True)
    CumulativeTravelTime = models.FloatField(null=True)
    PriorityOrder = models.PositiveIntegerField(null=True)

class EscortManagement(models.Model):
    EscortId = models.CharField(max_length=40,unique = True)
    EscortName = models.CharField(max_length=150)
    EscortProfilePicUpload = models.CharField(max_length=300,null = True)
    ContactNumber = models.CharField(max_length=20)
    Age = models.IntegerField()
    AadharCardUpload = models.CharField(max_length=300)
    Address = models.CharField(max_length=300)
    CertificationUpload = models.CharField(max_length=300,null = True)
    AccountHandlerName = models.CharField(max_length=255,null = True)
    AccountNumber = models.CharField(max_length=50,null = True)
    BankName = models.CharField(max_length=100,null = True)
    IFSCCode = models.CharField(max_length=40,null = True)
    BranchName = models.TextField(null = True)
    Shift = models.CharField(max_length = 20,null = True)
    ShiftStartTime = models.TimeField()
    ShiftEndTime = models.TimeField()
    EscortAddedDate = models.DateTimeField(auto_now_add = True)


class Drop(models.Model):
    BookingId = models.CharField(max_length=100, unique=True)
    EmployeeId = models.CharField(max_length=100)
    TripDate = models.DateField(null=True)
    LoginTime = models.CharField(max_length=100)
    LogoutTime = models.CharField(max_length=100)
    EmployeeName = models.CharField(max_length=200)
    EmployeeGender = models.CharField(max_length=10)
    EmployeeAddress = models.TextField()
    EmployeeCity = models.CharField(max_length=100)
    Latitude = models.FloatField()
    Longitude = models.FloatField()
    VehicleNumber = models.CharField(max_length=100, null=True)
    SeatCapacity = models.IntegerField(null=True)
    VehicleId = models.CharField(max_length=100,null=True)
    CumulativeTravelTime = models.FloatField(null=True)
    PriorityOrder = models.PositiveIntegerField(null=True)
    EscortId = models.CharField(max_length=100,null=True)

class PickUp(models.Model):
    BookingId = models.CharField(max_length=100,unique=True)
    EmployeeId = models.CharField(max_length=100)
    TripDate = models.DateField(null=True)
    LoginTime = models.CharField(max_length=100)
    LogoutTime = models.CharField(max_length=100)
    EmployeeName = models.CharField(max_length=200)
    EmployeeGender = models.CharField(max_length=10)
    EmployeeAddress = models.TextField()
    EmployeeCity = models.CharField(max_length=100)
    Latitude = models.FloatField()
    Longitude = models.FloatField()
    VehicleNumber = models.CharField(max_length=255, null=True, blank=True)
    SeatCapacity = models.IntegerField(null=True, blank=True)
    VehicleId = models.CharField(max_length=100)
    CumulativeTravelTime = models.FloatField(null=True)
    PriorityOrder = models.IntegerField(null=True)
