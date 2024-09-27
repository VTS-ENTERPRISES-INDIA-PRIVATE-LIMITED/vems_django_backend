import json
import copy
import math
import requests
import numpy as np
import pandas as pd
from geopy.distance import geodesic, great_circle
from geopy.distance import geodesic
import polyline
from .models import Trip,VehicleDetail,Result
from django.http import HttpResponse

def load_data():
    trips = Trip.objects.all()
    vehicle_details = VehicleDetail.objects.all()
    
    # Convert data to dataframes
    data = pd.DataFrame(list(trips.values()))
    vehicle_detail_df = pd.DataFrame(list(vehicle_details.values()))
    
    return data, vehicle_detail_df

def prepare_cab_details(cab_details_df):
    cab_number_dic = {}
    for index, row in cab_details_df.iterrows():
        if row['seat_capacity'] in cab_number_dic:
            cab_number_dic[row['seat_capacity']].append(row['vehicle_number'])
        else:
            cab_number_dic[row['seat_capacity']] = [row['vehicle_number']]
    capacity_dic = {}
    lis = cab_details_df['seat_capacity'].to_list()
    lis_set = list(set(lis))
    for i in set(lis_set):
        capacity_dic[i] = lis.count(i)
    return cab_number_dic, capacity_dic

def filter_data(data):
    data = data.dropna(how='all')
    center_latitude = 12.9775
    center_longitude = 80.2518
    radius_km = 25
    data['within_radius'] = data.apply(lambda row: is_within_radius(row, center_latitude, center_longitude, radius_km), axis=1)
    data = data[data['within_radius']]
    data = data.drop(columns=['within_radius'])
    return data

def is_within_radius(row, center_lat, center_lon, radius):
    center_point = (center_lat, center_lon)
    target_point = (row['latitude'], row['longitude'])
    distance = great_circle(center_point, target_point).km
    return distance <= radius

def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    bearing = (initial_bearing + 360) % 360
    return bearing

def group_by_direction(bearing):
    if 315 <= bearing or bearing < 45:
        return 'North'
    elif 45 <= bearing < 135:
        return 'East'
    elif 135 <= bearing < 225:
        return 'South'
    else:
        return 'West'

def get_directional_data(data):
    locations_for_grouping = {}
    for index, row in data.iterrows():
        if row['booking_id'] not in locations_for_grouping:
            locations_for_grouping[row['booking_id']] = (row['latitude'], row['longitude'])
        else:
            locations_for_grouping[row['booking_id']].append((row['latitude'], row['longitude']))
    office_lat, office_lon = 12.9775, 80.2518
    groups = {'North': [], 'East': [], 'South': [], 'West': []}
    for booking_id, (lat, lon) in locations_for_grouping.items():
        bearing = calculate_bearing(office_lat, office_lon, lat, lon)
        direction = group_by_direction(bearing)
        groups[direction].append(booking_id)
    return groups

def get_directional_dfs(data, groups):
    dfs = {}
    for direction, booking_ids in groups.items():
        direction_df = data[data['booking_id'].isin(booking_ids)]
        direction_df = direction_df.reset_index(drop=True)
        dfs[direction] = direction_df
    return dfs

def calculate_distance(row, reference_location):
    return geodesic((row['latitude'], row['longitude']), reference_location).km

def get_driving_distance(start_point, end_point):
    api_key = 'bb048d25-b15a-4b0b-aab1-4b9be2ab4f55'
    url = f"https://graphhopper.com/api/1/route?point={start_point[1]},{start_point[0]}&point={end_point[1]},{end_point[0]}&vehicle=car&locale=en&key={api_key}&points_encoded=false"
    response = requests.get(url)
    if response.status_code == 200:
        data = response.json()
        distance = data['paths'][0]['distance'] / 1000
        return distance
    else:
        print(f"Error in distance calculation: {response.status_code}")
        return None

def ride_allocation(employee_df, capacity_dic, cab_number_dic, first_distances):
    employee_locations = [[row['latitude'], row['longitude']] for _, row in employee_df.iterrows()]
    assigned_employees = []
    cabs_assigning = {}
    while len(assigned_employees) < len(first_distances) and sum(capacity_dic.values()) > 0:
        remaining_distances = [(i, d) for i, d in enumerate(first_distances) if i not in assigned_employees]
        if not remaining_distances:
            break
        farthest_employee_index, farthest_distance = max(remaining_distances, key=lambda x: x[1])
        farthest_employee_location = employee_locations[farthest_employee_index]
        office_location = [12.9775, 80.2518]
        start_lat, start_lon = office_location
        end_lat, end_lon = farthest_employee_location
        osrm_url = f"http://router.project-osrm.org/route/v1/driving/{start_lon},{start_lat};{end_lon},{end_lat}?overview=full"
        response = requests.get(osrm_url)
        if response.status_code == 200:
            data = response.json()
            route_geometry = data['routes'][0]['geometry']
            decoded_route = polyline.decode(route_geometry)
            sorted_capacity_dic = sorted(capacity_dic.items(), key=lambda x: int(x[0]))
            nearby_employees = []
            for i, employee_location in enumerate(employee_locations):
                if i not in assigned_employees and i != farthest_employee_index:
                    for route_point in decoded_route:
                        distance_geo = geodesic(route_point, employee_location).km
                        if distance_geo <= 1.5:
                            nearby_employees.append(i)
                            break
            total_employees = [farthest_employee_index] + nearby_employees
            assigned = False
            for capacity, cabs in sorted_capacity_dic:
                capacity_size = int(capacity)
                if len(total_employees) == capacity_size and cabs > 0:
                    cab_number_ = cab_number_dic[capacity_size][0]
                    cabs_assigning[cab_number_] = total_employees
                    capacity_dic[capacity] -= 1
                    cab_number_dic[capacity].pop(0)
                    assigned_employees.extend(total_employees)
                    assigned = True
                    break
            if not assigned:
                for capacity, cabs in sorted_capacity_dic:
                    capacity_size = int(capacity)
                    if len(total_employees) > capacity_size or cabs < 1:
                        continue
                    elif len(total_employees) < capacity_size and cabs > 0:
                        cab_number_ = cab_number_dic[capacity_size][0]
                        cabs_assigning[cab_number_] = total_employees[:capacity_size]
                        print(capacity_dic)
                        capacity_dic[capacity_size] -= 1
                        cab_number_dic[capacity_size].pop(0)
                        assigned_employees.extend(total_employees[:capacity_size])
                        total_employees = total_employees[capacity_size:]
                        assigned = True
                        break
        else:
            print(f"Error retrieving route geometry: {response.status_code}")
            break
    return cabs_assigning, capacity_dic, cab_number_dic
import functools

def ride(request):
    data, cab_details_df = load_data()
    cab_number_dic, capacity_dic = prepare_cab_details(cab_details_df)
    data = filter_data(data)
    groups = get_directional_data(data)
    dfs = get_directional_dfs(data, groups)
    merged_dfs = []
    for direction, df in dfs.items():
        df_locations = [[row['longitude'], row['latitude']] for _, row in df.iterrows()]
        distances = [get_driving_distance([80.2518, 12.9775], loc) for loc in df_locations]
        cabs, capacity_dic, cab_number_dic = ride_allocation(df, capacity_dic, cab_number_dic, distances)
        df['vehicle_number'] = None
        for cab_number, indices in cabs.items():
            for index in indices:
                df.at[index, 'vehicle_number'] = cab_number
        df = pd.merge(df, cab_details_df[['vehicle_number', 'seat_capacity', 'vehicleId']], on='vehicle_number', how='left')
        merged_dfs.append(df)
    combined_df = pd.concat(merged_dfs, ignore_index=True)
    for index, row in combined_df.iterrows():
        result = Result(
            booking_id=row['booking_id'],
            employee_id=row['employee_id'],
            date=row['date'],
            in_time=row['in_time'],
            out_time=row['out_time'],
            employee_name=row['employee_name'],
            gender=row['gender'],
            address=row['address'],
            city=row['city'],
            latitude=row['latitude'],
            longitude=row['longitude'],
            vehicle_number=row['vehicle_number'],
            seat_capacity=row['seat_capacity'],
            vehicle_id=row['vehicleId']
        )
        result.save()
    return HttpResponse({"success":"done"}, status=200)




#2222222222222
import json
import copy
import math
import requests
import numpy as np
import pandas as pd
from geopy.distance import geodesic, great_circle
from geopy.distance import geodesic
import polyline
from .models import Trip, VehicleDetail, Result,Ride_histories
from django.http import HttpResponse
from django.db import transaction
import logging
from rest_framework.response import Response
from rest_framework import status
from rest_framework.decorators import api_view
from django.http import JsonResponse


logging.basicConfig(level=logging.INFO)
office_lat, office_lon = 12.9775, 80.2518


def load_data():
    try:
        trips = Trip.objects.all()
        vehicle_details = VehicleDetail.objects.all()
        data = pd.DataFrame(list(trips.values()))
        vehicle_detail_df = pd.DataFrame(list(vehicle_details.values()))
        return data, vehicle_detail_df
    except Exception as e:
        logging.error(f"Error loading data contact Db Team: {e}")
        return None, None

def prepare_cab_details(cab_details_df):
    try:
        cab_number_dic = {}
        for index, row in cab_details_df.iterrows():
            if row['seat_capacity'] in cab_number_dic:
                cab_number_dic[row['seat_capacity']].append(row['vehicle_number'])
            else:
                cab_number_dic[row['seat_capacity']] = [row['vehicle_number']]
        capacity_dic = {}
        lis = cab_details_df['seat_capacity'].to_list()
        lis_set = list(set(lis))
        for i in set(lis_set):
            capacity_dic[i] = lis.count(i)
        return cab_number_dic, capacity_dic
    except Exception as e:
        logging.error(f"Error preparing cab details: {e}")
        return None, None

def filter_data(data):
    try:
        data = data.dropna(how='all')
        center_latitude = office_lat
        center_longitude = office_lon
        radius_km = 25
        data['within_radius'] = data.apply(lambda row: is_within_radius(row, center_latitude, center_longitude, radius_km), axis=1)
        data = data[data['within_radius']]
        data = data.drop(columns=['within_radius'])
        return data
    except Exception as e:
        logging.error(f"Error filtering data: {e}")
        return None

def is_within_radius(row, center_lat, center_lon, radius):
    try:
        center_point = (center_lat, center_lon)
        target_point = (row['latitude'], row['longitude'])
        distance = great_circle(center_point, target_point).km
        return distance <= radius
    except Exception as e:
        logging.error(f"Error calculating distance: {e}")
        return False

def calculate_bearing(lat1, lon1, lat2, lon2):
    try:
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        bearing = (initial_bearing + 360) % 360
        return bearing
    except Exception as e:
        logging.error(f"Error calculating bearing: {e}")
        return None

def group_by_direction(bearing):
    try:
        if 315 <= bearing or bearing < 45:
            return 'North'
        elif 45 <= bearing < 135:
            return 'East'
        elif 135 <= bearing < 225:
            return 'South'
        else:
            return 'West'
    except Exception as e:
        logging.error(f"Error grouping by direction: {e}")
        return None

def get_directional_data(data):
    try:
        locations_for_grouping = {}
        for index, row in data.iterrows():
            if row['booking_id'] not in locations_for_grouping:
                locations_for_grouping[row['booking_id']] = (row['latitude'], row['longitude'])
            else:
                locations_for_grouping[row['booking_id']].append((row['latitude'], row['longitude']))
        #office_lat, office_lon = 12.9775, 80.2518
        groups = {'North': [], 'East': [], 'South': [], 'West': []}
        for booking_id, (lat, lon) in locations_for_grouping.items():
            bearing = calculate_bearing(office_lat, office_lon, lat, lon)
            direction = group_by_direction(bearing)
            groups[direction].append(booking_id)
        return groups
    except Exception as e:
        logging.error(f"Error getting directional data: {e}")
        return None

def get_directional_dfs(data, groups):
    try:
        dfs = {}
        for direction, booking_ids in groups.items():
            direction_df = data[data['booking_id'].isin(booking_ids)]
            direction_df = direction_df.reset_index(drop=True)
            dfs[direction] = direction_df
        return dfs
    except Exception as e:
        logging.error(f"Error getting directional dfs: {e}")
        return None

def calculate_distance(row, reference_location):
    try:
        return geodesic((row['latitude'], row['longitude']), reference_location).km
    except Exception as e:
        logging.error(f"Error calculating distance: {e}")
        return None

def get_driving_distance(start_point, end_point):
    try:
        api_key = 'bb048d25-b15a-4b0b-aab1-4b9be2ab4f55'
        url = f"https://graphhopper.com/api/1/route?point={start_point[1]},{start_point[0]}&point={end_point[1]},{end_point[0]}&vehicle=car&locale=en&key={api_key}&points_encoded=false"
        response = requests.get(url, timeout=10)
        if response.status_code == 200:
            data = response.json()
            distance = data['paths'][0]['distance'] / 1000
            return distance
        else:
            logging.error(f"Error retrieving driving distance: {response.status_code}")
            return None
    except requests.exceptions.Timeout:
        logging.error("Timeout error occurred while retrieving driving distance")
        return None
    except Exception as e:
        logging.error(f"Error getting driving distance: {e}")
        return None

def ride_allocation(employee_df, capacity_dic, cab_number_dic, first_distances):
    try:
        employee_locations = [[row['latitude'], row['longitude']] for _, row in employee_df.iterrows()]
        assigned_employees = []
        cabs_assigning = {}
        while len(assigned_employees) < len(first_distances) and sum(capacity_dic.values()) > 0:
            remaining_distances = [(i, d) for i, d in enumerate(first_distances) if i not in assigned_employees]
            if not remaining_distances:
                break
            farthest_employee_index, farthest_distance = max(remaining_distances, key=lambda x: x[1])
            farthest_employee_location = employee_locations[farthest_employee_index]
            office_location = [office_lat, office_lon]
            start_lat, start_lon = office_location
            end_lat, end_lon = farthest_employee_location
            osrm_url = f"http://router.project-osrm.org/route/v1/driving/{start_lon},{start_lat};{end_lon},{end_lat}?overview=full"
            response = requests.get(osrm_url, timeout=10)
            if response.status_code == 200:
                data = response.json()
                route_geometry = data['routes'][0]['geometry']
                decoded_route = polyline.decode(route_geometry)
                sorted_capacity_dic = sorted(capacity_dic.items(), key=lambda x: int(x[0]))
                nearby_employees = []
                for i, employee_location in enumerate(employee_locations):
                    if i not in assigned_employees and i != farthest_employee_index:
                        for route_point in decoded_route:
                            distance_geo = geodesic(route_point, employee_location).km
                            if distance_geo <= 1.5:
                                nearby_employees.append(i)
                                break
                total_employees = [farthest_employee_index] + nearby_employees
                assigned = False
                for capacity, cabs in sorted_capacity_dic:
                    capacity_size = int(capacity)
                    if len(total_employees) == capacity_size and cabs > 0:
                        cab_number_ = cab_number_dic[capacity_size][0]
                        cabs_assigning[cab_number_] = total_employees
                        capacity_dic[capacity] -= 1
                        cab_number_dic[capacity].pop(0)
                        assigned_employees.extend(total_employees)
                        assigned = True
                        break
                if not assigned:
                    for capacity, cabs in sorted_capacity_dic:
                        capacity_size = int(capacity)
                        if len(total_employees) > capacity_size or cabs < 1:
                            continue
                        elif len(total_employees) < capacity_size and cabs > 0:
                            cab_number_ = cab_number_dic[capacity_size][0]
                            cabs_assigning[cab_number_] = total_employees[:capacity_size]
                            capacity_dic[capacity_size] -= 1
                            cab_number_dic[capacity_size].pop(0)
                            assigned_employees.extend(total_employees[:capacity_size])
                            total_employees = total_employees[capacity_size:]
                            assigned = True
                            break
            else:
                logging.error(f"Error retrieving route geometry: {response.status_code}")
                break
        return cabs_assigning, capacity_dic, cab_number_dic
    except Exception as e:
        logging.error(f"Error in ride allocation: {e}")
        return None, None, None

def ride(request):
    try:
        data, cab_details_df = load_data()
        cab_number_dic, capacity_dic = prepare_cab_details(cab_details_df)
        data = filter_data(data)
        groups = get_directional_data(data)
        dfs = get_directional_dfs(data, groups)
        merged_dfs = []
        for direction, df in dfs.items():
            df_locations = [[row['longitude'], row['latitude']] for _, row in df.iterrows()]
            distances = [get_driving_distance([office_lon, office_lat], loc) for loc in df_locations]
            cabs, capacity_dic, cab_number_dic = ride_allocation(df, capacity_dic, cab_number_dic, distances)
            df['vehicle_number'] = None
            for cab_number, indices in cabs.items():
                for index in indices:
                    df.at[index, 'vehicle_number'] = cab_number
            df = pd.merge(df, cab_details_df[['vehicle_number', 'seat_capacity', 'vehicleId']], on='vehicle_number', how='left')
            merged_dfs.append(df)
        combined_df = pd.concat(merged_dfs, ignore_index=True)
        for index, row in combined_df.iterrows():
            try:
                result = Result(
                    booking_id=row['booking_id'],
                    employee_id=row['employee_id'],
                    date=row['date'],
                    in_time=row['in_time'],
                    out_time=row['out_time'],
                    employee_name=row['employee_name'],
                    gender=row['gender'],
                    address=row['address'],
                    city=row['city'],
                    latitude=row['latitude'],
                    longitude=row['longitude'],
                    vehicle_number=row['vehicle_number'],
                    seat_capacity=row['seat_capacity'],
                    vehicle_id=row['vehicleId']
                )
                result.save()
                cumulative_time(result)
            except Exception as e:
                logging.error(f"Error saving result: {e}")
                if Result.objects.filter(booking_id=row['booking_id']).exists():
                    logging.info(f"Booking ID {row['booking_id']} already allocated.....!")
        return JsonResponse({"success": "Rides allocated"}, status=status.HTTP_200_OK)

    except Exception as e:
        logging.error(f"Error in ride function: {e}")
        return JsonResponse({"error": "Allocation Failed"}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)

def train_new_data(request):
    try:
        with transaction.atomic():
            results_data = Result.objects.all()

            histories_data = [
                Ride_histories(
                    booking_id=item.booking_id,
                    employee_id=item.employee_id,
                    date=item.date,
                    in_time=item.in_time,
                    out_time=item.out_time,
                    employee_name=item.employee_name,
                    gender=item.gender,
                    address=item.address,
                    city=item.city,
                    latitude=item.latitude,
                    longitude=item.longitude,
                    vehicle_number=item.vehicle_number,
                    seat_capacity=item.seat_capacity,
                    vehicle_id=item.vehicle_id
                ) for item in results_data
            ]
            Ride_histories.objects.bulk_create(histories_data)
            Result.objects.all().delete()
        return JsonResponse({"success": "Data update & Old data moved to Bin"}, status=status.HTTP_200_OK)
    except Exception as e:
        logging.error(f"Error in new data insertion: {e}")
        return JsonResponse({"error": "Failed data update"}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)


def cumulative_time(result):
    try:
        api_key = "AIzaSyABQaAOBDJ9w0nH8w1RfpK3bilCVvn1cxY"
        vehicle_id = result.vehicle_id
        results_with_same_vehicle_id = Result.objects.filter(vehicle_id=vehicle_id)
        cumulative_travel_times = []
        prev_lat = office_lat
        prev_lon = office_lon
        cumulative_time = 0
        for result_with_same_vehicle_id in results_with_same_vehicle_id:
            try:
                travel_time = calculate_travel_time(result_with_same_vehicle_id, prev_lat, prev_lon, api_key)
                cumulative_time += travel_time
                cumulative_travel_times.append(cumulative_time)
                prev_lat = result_with_same_vehicle_id.latitude
                prev_lon = result_with_same_vehicle_id.longitude
            except Exception as e:
                logging.error(f"Error calculating travel time for result {result_with_same_vehicle_id.id}: {e}")
        try:
            result.CumulativeTravelTime = cumulative_travel_times[-1]
            result.save()
        except IndexError:
            logging.error(f"No cumulative travel times calculated for result {result.id}")
    except Exception as e:
        logging.error(f"Error processing result {result.id}: {e}")


def calculate_travel_time(result, center_lat, center_lon, api_key):
    try:
        # Calculate travel time between two points using Google Distance Matrix API
        url = "https://maps.googleapis.com/maps/api/distancematrix/json"
        params = {
            'origins': f'{center_lat},{center_lon}',
            'destinations': f'{result.latitude},{result.longitude}',
            'key': api_key,
            'mode': 'driving'
        }
        response = requests.get(url, params=params)
        response.raise_for_status()
        response_json = response.json()
        try:
            duration = response_json['rows'][0]['elements'][0]['duration']['value']
            return duration / 60
        except KeyError as ee:
            logging.error(f"Error Duration not get: {ee}")
            return None
    except requests.exceptions.RequestException as e:
        logging.error(f"Error in making API req: {e}")
        return None
    except Exception as e:
        logging.error(f"Error in calc time: {e}")
        return None

@api_view(['GET'])
def train_and_ride(request):
    try:
        response = train_new_data(request)
        if response.status_code == 200:
            print("ride allocation under processing...")
            return ride(request)
        else:
            return Response({"error": "Failed data update"}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)
    except Exception as e:
        logging.error(f"Error in train_and_ride: {e}")
        return Response({"error": "Failed Retrain & update allocation"}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)

# bofe edit the code please make a copy it..!



import io
import csv
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_http_methods


@csrf_exempt
@require_http_methods(["POST"])
def upload_trips(request):
    if 'file' not in request.FILES:
        return JsonResponse({'status': 'error', 'message': 'No file uploaded'}, status=400)
    
    file = request.FILES['file']
    
    if not (file.name.endswith('.csv') or file.name.endswith('.xlsx')):
        return JsonResponse({'status': 'error', 'message': 'File must be CSV or XLSX type'}, status=400)
    
    try:
        if file.name.endswith('.csv'):
            file_data = file.read().decode('utf-8-sig')
            csv_data = io.StringIO(file_data)
            reader = csv.DictReader(csv_data)
        else:
            df = pd.read_excel(file)
            reader = df.to_dict(orient='records')

        for row in reader:
            Trip.objects.create(
                booking_id=row['Booking ID'],
                latitude=float(row['Latitude']),
                longitude=float(row['Longitude']),
                employee_id=row['Employee ID'],
                date=pd.to_datetime(row['Date'], format='%m/%d/%Y').strftime('%Y-%m-%d'),
                in_time=pd.to_datetime(row['In Time'], format='%H:%M:%S').time(),  # Adjust format here
                out_time=pd.to_datetime(row['Out Time'], format='%H:%M:%S').time(),
                employee_name=row['Employee Name'],
                gender=row['Gender'],
                address=row['Address'],
                city=row['City']
            )

        return JsonResponse({'status': 'success', 'message': 'File processed successfully'}, status=200)
    
    except Exception as e:
        return JsonResponse({'status': 'error', 'message': str(e)}, status=400)

@csrf_exempt
@require_http_methods(["POST"])
def upload_vehicles(request):
    if 'file' not in request.FILES:
        return JsonResponse({'status': 'error', 'message': 'No file uploaded'}, status=400)
    
    file = request.FILES['file']
    
    if not (file.name.endswith('.csv') or file.name.endswith('.xlsx')):
        return JsonResponse({'status': 'error', 'message': 'File must be CSV or XLSX type'}, status=400)
    
    try:
        if file.name.endswith('.csv'):
            file_data = file.read().decode('utf-8')
            csv_data = io.StringIO(file_data)
            reader = csv.DictReader(csv_data)
        else:
            df = pd.read_excel(file)
            reader = df.to_dict(orient='records')

        for row in reader:
            VehicleDetail.objects.create(
                vehicleId=row['vehicleId'],
                vehicle_number=row['vehicleNumber'],
                seat_capacity=int(row['seatCapacity']),
                vehicleName=row['vehicleName'],
                vehicleType=row['vehicleType'],
                vendorName=row['vendorName'],
                insuranceNumber=row['insuranceNumber'],
                mileage=float(row['mileage']),
                yearOfManufacturing=int(row['yearOfManufacturing']),
                fuelType=row['fuelType'],
                vehicleImage=row['vehicleImage'],
                driverId=row.get('driverId', None),
                vehicleStatus=int(row['vehicleStatus'])
            )

        return JsonResponse({'status': 'success', 'message': 'File processed successfully'}, status=200)
    
    except Exception as e:
        return JsonResponse({'status': 'error', 'message': str(e)}, status=400)