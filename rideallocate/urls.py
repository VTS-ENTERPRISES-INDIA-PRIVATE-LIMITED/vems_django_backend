from django.urls import path
from .views import ride,train_new_data,train_and_ride,main_view
from.utils import upload_trips,upload_vehicles
urlpatterns = [
    #Don't fetch this urls
    path('test_allocate',ride,name = 'ride'), 
    path('test_re_asssign',train_new_data,name = 'train_new_data'),
    path('up_trip_csv',upload_trips,name='upload_trip_csv'),
    path('up_veh_csv',upload_vehicles,name='upload_vehicle_csv'),

    #Main Url's
    path('allocate/',train_and_ride,name='train_and_ride'),
    path('escorts/', main_view, name='ride-allocation'),
]