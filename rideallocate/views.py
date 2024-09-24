import json
import copy
import math
import requests
import numpy as np
import pandas as pd
from geopy.distance import geodesic, great_circle
from geopy.distance import geodesic
import polyline
from datetime import datetime, time
from django.db.models import Q
from .models import Trip, VehiclesData, PickUpData,Ride_histories
from .models import EscortManagement, CabAllocation, DropingData,Histories
from django.http import HttpResponse
from django.db import transaction
import logging
from rest_framework.response import Response
from rest_framework import status
from rest_framework.decorators import api_view
from django.http import JsonResponse
from django.conf import settings


logging.basicConfig(level=logging.INFO)
office_lat, office_lon = 12.9775, 80.2518
graphhopper_api_key_1 = settings.GRAPHOPPER_API_KEY_1
google_api_key_1 = settings.GOOGLE_API_KEY_1


def load_data():
    try:
        trips = Trip.objects.all()
        vehicle_details = VehiclesData.objects.all()
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
            if row['SeatCapacity'] in cab_number_dic:
                cab_number_dic[row['SeatCapacity']].append(row['VehicleNumber'])
            else:
                cab_number_dic[row['SeatCapacity']] = [row['VehicleNumber']]
        capacity_dic = {}
        lis = cab_details_df['SeatCapacity'].to_list()
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
        api_key = graphhopper_api_key_1
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
            df['VehicleNumber'] = None
            for cab_number, indices in cabs.items():
                for index in indices:
                    df.at[index, 'VehicleNumber'] = cab_number
            df = pd.merge(df, cab_details_df[['VehicleNumber', 'SeatCapacity', 'VehicleId']], on='VehicleNumber', how='left')
            merged_dfs.append(df)
        combined_df = pd.concat(merged_dfs, ignore_index=True)
        for index, row in combined_df.iterrows():
            try:
                result = PickUpData(
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
                    VehicleNumber=row['VehicleNumber'],
                    SeatCapacity=row['SeatCapacity'],
                    vehicle_id=row['VehicleId']
                )
                result.save()
                #cumulative_time(result)
            except Exception as e:
                logging.error(f"Error saving result: {e}")
                if PickUpData.objects.filter(booking_id=row['booking_id']).exists():
                    logging.info(f"Booking ID {row['booking_id']} already allocated.....!")

        return JsonResponse({"success": "Rides allocated"}, status=status.HTTP_200_OK)

    except Exception as e:
        logging.error(f"Error in ride function: {e}")
        return JsonResponse({"error": "Allocation Failed"}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)

def train_new_data(request):
    try:
        with transaction.atomic():
            results_data = PickUpData.objects.all()

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
                    VehicleNumber=item.VehicleNumber,
                    SeatCapacity=item.SeatCapacity,
                    vehicle_id=item.vehicle_id,
                    CumulativeTravelTime = item.CumulativeTravelTime,
                    priority_order = item.priority_order
                ) for item in results_data
            ]
            PickUpData.objects.all().delete()
        return JsonResponse({"success": "Data update & Old data moved to Bin"}, status=status.HTTP_200_OK)
    except Exception as e:
        logging.error(f"Error in new data insertion: {e}")
        return JsonResponse({"error": "Failed data update"}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)


@api_view(['GET'])
def train_and_ride(request):
    try:
        """results = Result.objects.all().values(
            'booking_id', 
            'employee_id', 
            'date', 
            'in_time', 
            'out_time', 
            'employee_name', 
            'gender', 
            'address', 
            'city', 
            'latitude', 
            'longitude', 
            'VehicleNumber', 
            'SeatCapacity', 
            'vehicle_id',
            'CumulativeTravelTime',
            'priority_order'
        )

        results_list = list(results)
        df = pd.DataFrame(results_list)
        csv_file_path = 'results_output.csv'
        df.to_csv(csv_file_path, index=False)

        print(f"Data successfully written to {csv_file_path}")"""
        response = train_new_data(request)
        if response.status_code == 200:
            print("ride allocation under processing...")
            ride_res = ride(request)
            if ride_res.status_code ==200:
                print("priority & cumulative time calcs under processing...")
                get_priority_and_cumulative_time()
                return JsonResponse({"success": "Rides allocated"}, status=status.HTTP_200_OK)
        else:
            return Response({"error": "Failed data update"}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)
    except Exception as e:
        logging.error(f"Error in train_and_ride: {e}")
        return Response({"error": "Failed Retrain & update allocation"}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)

def get_driving_distance(start_point, end_point):
    """
    Returns the driving distance between two points using the GraphHopper API.
    """
    api_key = graphhopper_api_key_1
    url = f"https://graphhopper.com/api/1/route?point={start_point[1]},{start_point[0]}&point={end_point[1]},{end_point[0]}&vehicle=car&locale=en&key={api_key}&points_encoded=false"

    try:
        response = requests.get(url)
        response.raise_for_status()
        data = response.json()
        distance = data['paths'][0]['distance'] / 1000  # Convert meters to kilometers
        return distance
    except requests.exceptions.RequestException as e:
        logging.error(f"Error in distance calculation: {e}")
        return None

def get_optimal_route_for_vehicles(df):
    """
    Returns the optimal route for vehicles based on employee locations and out times.
    """
    api_key = graphhopper_api_key_1
    graphhopper_url = f"https://graphhopper.com/api/1/vrp/optimize?key={api_key}"

    office_location = [80.2518, 12.9775]

    vehicle_ids = df['vehicle_id'].unique()
    result = {}

    for vehicle_id in vehicle_ids:
        filtered_df = df[df['vehicle_id'] == vehicle_id]

        employee_locations = filtered_df[['longitude', 'latitude']].values.tolist()
        employee_ids = filtered_df['employee_id'].tolist()
        genders = filtered_df['gender'].tolist()
        out_times = filtered_df['out_time'].tolist()

        distances_from_office = [
            get_driving_distance(office_location, loc) for loc in employee_locations
        ]

        if len(employee_ids) == 1:
            # Handle the case with only one employee
            emp_id = employee_ids[0]
            gender = genders[0]
            location = employee_locations[0]
            out_time = out_times[0]

            result[f'{vehicle_id}'] = [{
                "emp_id": emp_id,
                "gender": gender,
                "priority_order": 1,
                "latitude": location[1],
                "longitude": location[0],
                "out_time": out_time
            }]
            continue

        farthest_index = distances_from_office.index(max(distances_from_office))

        # Create the list of employees including the farthest one
        employee_data = list(zip(employee_ids, genders, employee_locations, out_times))

        # Separate the farthest employee
        farthest_employee = employee_data[farthest_index]
        other_employees = [data for i, data in enumerate(employee_data) if i != farthest_index]

        # Create the optimization request body
        optimization_body = {
            "jobs": [
                {"id": i + 1, "location": loc} for i, (emp_id, gender, loc, outtime) in enumerate(other_employees)
            ],
            "vehicles": [
                {
                    "id": 1,
                    "start": farthest_employee[2],
                    "end": office_location,
                    "profile": "driving-car"
                }
            ]
        }

        headers = {
            'Accept': 'application/json, application/geo+json, application/gpx+xml, img/png; charset=utf-8',
            'Authorization': '5b3ce3597851110001cf6248733deb06b5f5402da55b7ceff83ec234',
            'Content-Type': 'application/json; charset=utf-8'
        }

        try:
            optimization_response = requests.post('https://api.openrouteservice.org/optimization', json=optimization_body, headers=headers)
            optimization_response.raise_for_status()
            data = optimization_response.json()
            optimized_route = data['routes'][0]['steps']
            cab_data = [{
                "emp_id": farthest_employee[0],
                "gender": farthest_employee[1],
                "priority_order": 1,
                "latitude": farthest_employee[2][1],
                "longitude": farthest_employee[2][0],
                "out_time": farthest_employee[3]
            }]
            for i, step in enumerate(optimized_route):
                job_id = step.get('job', None)
                if job_id:
                    emp_id, gender, location, outtime = other_employees[job_id - 1]

                    cab_data.append({
                        "emp_id": emp_id,
                        "gender": gender,
                        "priority_order": i + 1,
                        "latitude": location[1],
                        "longitude": location[0],
                        "out_time": outtime
                    })

            result[f'{vehicle_id}'] = cab_data
        except requests.exceptions.RequestException as e:
            logging.error(f"Error for vehicle ID {vehicle_id}: {e}")
            result[f'{vehicle_id}'] = []
    return result

def get_priority():
    try:
        result = PickUpData.objects.all()
        data_ = pd.DataFrame(list(result.values()))
        result = get_optimal_route_for_vehicles(data_)
        result_objs = []
        for vehicle_id, employees in result.items():
            for employee in employees:
                emp_id = employee['emp_id']
                priority_order = employee['priority_order']

                result_obj = PickUpData.objects.get(employee_id=emp_id)
                result_obj.priority_order = priority_order
                result_objs.append(result_obj)

        with transaction.atomic():
            PickUpData.objects.bulk_update(result_objs, ['priority_order'])
        return True
    except Exception as e:
        logging.error(f"Error in get_priority: {e}")
        return False

def get_priority_and_cumulative_time():
    try:
        if get_priority():
            result_ = PickUpData.objects.all()
            data_ = pd.DataFrame(list(result_.values()))
            data = cumulative_time(data_,office_lat,office_lon)

            results_to_update = []
            for index, row in data.iterrows():
                try:
                    result_object = PickUpData.objects.get(booking_id=row['booking_id'])
                    if row['CumulativeTravelTime'] is None:
                        result_object.CumulativeTravelTime = None
                    else:
                        result_object.CumulativeTravelTime = row['CumulativeTravelTime']
                    results_to_update.append(result_object)
                except PickUpData.DoesNotExist:
                    logging.error(f"Result with booking_id {row['booking_id']} does not exist.")
                except Exception as e:
                    logging.error(f"Error updating result for booking_id {row['booking_id']}: {e}")
            PickUpData.objects.bulk_update(results_to_update, fields=['CumulativeTravelTime'])
            
        else:
            raise Exception("Failed to allocate priorities & time")
    except Exception as e:
        logging.error(f"Error in get_priority & cumulative time finding:{e}")
        raise

def function_for_travelling_time(row, center_lat, center_lon):
    
    url = "https://maps.googleapis.com/maps/api/distancematrix/json"
    api_key = google_api_key_1

    try:
        # Define the parameters
        params = {
            'origins': f'{center_lat},{center_lon}',
            'destinations': f'{row["latitude"]},{row["longitude"]}',
            'key': api_key,
            'mode': 'driving'  # Specify the mode of transport (driving, walking, etc.)
        }

        # Make the GET request
        response = requests.get(url, params=params)

        # Check if the request was successful
        if response.status_code == 200:
            response_json = response.json()
            if response_json['status'] == 'OK':
                try:
                    # Extract and print duration
                    duration = response_json['rows'][0]['elements'][0]['duration']['value']
                    duration_in_minutes = duration / 60
                    return duration_in_minutes
                except KeyError as e:
                    logging.error(f"Error at fun_ for travel time: The expected keys were not found in the response. {e}")
                    return None
            else:
                logging.error(f"Error fun_ for travel time: {response_json['error_message']}")
                return None
        else:
            logging.error(f"Error fun_ for travel time: {response.text}")
            return None
    except requests.exceptions.RequestException as e:
        logging.error(f"Error: An error occurred while making the request. {e}")
        return None
    except Exception as e:
        logging.error(f"Error: An unexpected error occurred. {e}")
        return None


def cumulative_time(data, center_lat, center_lon):
    try:
        data = data.sort_values(['VehicleNumber', 'priority_order'], ascending=[True, True])
        unique_vehicle_ids = data['vehicle_id'].unique()

        for vehicle_id in unique_vehicle_ids:
            vehicle_data = data[data['vehicle_id'] == vehicle_id]

            cumulative_travel_times = []

            prev_lat = center_lat
            prev_lon = center_lon
            cumulative_time = 0

            for index, row in vehicle_data.iterrows():
                travel_time = function_for_travelling_time(row, prev_lat, prev_lon)

                if travel_time is not None:
                    cumulative_time += travel_time
                    cumulative_travel_times.append(cumulative_time)
                    prev_lat = row['latitude']
                    prev_lon = row['longitude']
                else:
                    logging.warning(f"Warning: Travel time for vehicle {vehicle_id} could not be calculated.")

            #vehicle_data['CumulativeTravelTime'] = cumulative_travel_times
            vehicle_data.loc[:, 'CumulativeTravelTime'] = cumulative_travel_times
            data.loc[data['vehicle_id'] == vehicle_id, 'CumulativeTravelTime'] = vehicle_data['CumulativeTravelTime']

        return data
    except Exception as e:
        logging.error(f"Error: An unexpected error occurred. {e}")
        return None
#################################################################################################
#EScorts views from here
def convert_to_time(time_str):
    try:
        if isinstance(time_str, datetime.time):
            return time_str
        else:
            return datetime.strptime(time_str, "%H:%M:%S")
    except Exception as e:
        logging.error(f"Error at convert to time:{e}")

def find_available_escort(employee_out_time):
    try:
        if isinstance(employee_out_time, time):
            emp_out_time = employee_out_time
        else:
            emp_out_time = convert_to_time(employee_out_time)
        
        available_escorts = EscortManagement.objects.filter(
            Q(ShiftStartTime__lte=emp_out_time) & Q(ShiftEndTime__gte=emp_out_time)
        ).values_list('EscortId', flat=True)
        return list(available_escorts)
    except Exception as e:
        logging.error(f"Error at find availble escorts:{e}")

def haversine(lat1, lon1, lat2, lon2):
    try:
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371
        return c * r
    except Exception as e:
        logging.error(f"Error at haversime calculation:{e}")
        return None

def add_escort(escorts, index, employees):
    try:
        escort = {"escort_id": escorts[index].EscortId}
        employees.append(escort)
        return employees
    except Exception as e:
        logging.error(f"Error at add escort:{e}")
        return []

def process_cab_allocations():
    try:
        try:
            results_data = PickUpData.objects.all()
            if results_data.exists():
                with transaction.atomic():
                    cab_allocation_data = [
                        CabAllocation(
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
                            VehicleNumber=item.VehicleNumber,
                            SeatCapacity=item.SeatCapacity,
                            vehicle_id=item.vehicle_id,
                            CumulativeTravelTime=item.CumulativeTravelTime,
                            priority_order=item.priority_order
                        ) for item in results_data if item.priority_order is not None and item.CumulativeTravelTime is not None
                    ]

                    if cab_allocation_data:
                        CabAllocation.objects.bulk_create(cab_allocation_data)
                    else:
                        logging.info("No CabAllocation objects to create, Check Priorities & Times is availble in Result of Ride allocation")
                        logging.info("Process the Ride allocation first go--> 'shift_update/'")
                        return {'error': 'Error processing cab allocations'}
            else:
                logging.info("Perform Ride allocation before Escort assining...! fetch this URL shift_update/")
                return {'error': 'Error processing cab allocations'}
        except Exception as e:
            logging.error(f"Error in new data insertion: {e}")
            return JsonResponse({"error": "Failed data update"}, status=status.HTTP_500_INTERNAL_SERVER_ERROR)

        cab_vacant_details = VehiclesData.objects.filter(VehicleStatus=0)
        cab_details_df = VehiclesData.objects.all()
        escorts = EscortManagement.objects.all()
        data = CabAllocation.objects.all()

        cvd = pd.DataFrame(list(cab_vacant_details.values()))
        cd = pd.DataFrame(list(cab_details_df.values()))
        ed = pd.DataFrame(list(escorts.values()))
        dp = pd.DataFrame(list(data.values()))

        sorted_df = dp.sort_values(by=['priority_order'])
        data_ = sorted_df.groupby('vehicle_id').apply(lambda x: x.to_dict(orient='records')).to_dict()

        for vehicle_id, employees in data_.items():
            if isinstance(employees, list) and employees and 'priority_order' in employees[0]:
                max_priority = len(employees)
                for employee in employees:
                    employee['priority_order'] = (max_priority - int(employee['priority_order'])) + 1

        data = {key: list(reversed(value)) for key, value in data_.items()}

        cvd = cvd.sort_values(by=['SeatCapacity'])

        employee_out_time = list(data.items())[0][1][0]['out_time']
        result = find_available_escort(employee_out_time)
        escorts = escorts.filter(EscortId__in=result)

        index = 0
        items_to_process = list(data.items())
        for cab, employees in items_to_process:
            if all(emp['gender'].lower() == 'male' for emp in employees):
                continue
            cab_capacity = employees[0]['SeatCapacity']
            if all(emp['gender'].lower() == 'female' for emp in employees):
                if cab_capacity > len(employees):
                    employees = add_escort(escorts, index, employees)
                    data[cab] = employees
                    index += 1
                elif cab_capacity == len(employees):
                    larger_cabs = cvd[cvd['SeatCapacity'] > cab_capacity]
                    if not larger_cabs.empty:
                        new_cab_id = larger_cabs.iloc[0]['VehicleId']
                        employees = add_escort(escorts, index, employees)
                        data[new_cab_id] = employees
                        index += 1
                        cvd = cvd[cvd['VehicleId'] != new_cab_id]
                        cvd = pd.concat([cvd, cd[cd['VehicleId'] == cab]], ignore_index=True)
                        del data[cab]
                    elif larger_cabs.empty and not cvd.empty:
                        half = len(employees) // 2
                        extra_capacity_cabs = cvd[cvd['SeatCapacity'] > half]
                        extra_cab = extra_capacity_cabs.iloc[0]['VehicleId']

                        first_half_employees = employees[:half]
                        first_half_employees = add_escort(escorts, index, first_half_employees)
                        data[cab] = first_half_employees
                        index += 1

                        second_half_employees = employees[half:]
                        second_half_employees = add_escort(escorts, index, second_half_employees)
                        data[extra_cab] = second_half_employees
                        index += 1

                        cvd = cvd[cvd['VehicleId'] != extra_cab]
            else:
                last_employee = employees[-1]
                if last_employee['gender'].lower() == 'male':
                    continue
                last_female = next((emp for emp in reversed(employees) if emp['gender'].lower() == 'female'), None)
                last_male = next((emp for emp in reversed(employees) if emp['gender'].lower() == 'male'), None)

                if last_female and last_male:
                    distance = haversine(last_male["latitude"], last_male["longitude"], last_female["latitude"], last_female["longitude"])
                    if distance > 0.5:

                        if cab_capacity > len(employees):
                            employees = add_escort(escorts, index, employees)
                            data[cab] = employees
                            index += 1

                        elif cab_capacity == len(employees):
                            larger_cabs = cvd[cvd['SeatCapacity'] > cab_capacity]
                            if not larger_cabs.empty:
                                new_cab_id = larger_cabs.iloc[0]['VehicleId']
                                employees = add_escort(escorts, index, employees)
                                data[new_cab_id] = employees
                                index += 1
                                cvd = cvd[cvd['VehicleId'] != new_cab_id]
                                cvd = pd.concat([cvd, cd[cd['VehicleId'] == cab]], ignore_index=True)
                                del data[cab]
                            elif larger_cabs.empty and not cvd.empty:
                                half = len(employees) // 2
                                extra_capacity_cabs = cvd[cvd['SeatCapacity'] > half]
                                extra_cab = extra_capacity_cabs.iloc[0]['VehicleId']
                                second_half_employees = employees[half:]
                                second_half_employees = add_escort(escorts, index, second_half_employees)
                                data[cab] = second_half_employees
                                index += 1
                                first_half_employees = employees[:half]
                                if all(emp['gender'].lower() == 'female' for emp in first_half_employees):
                                    first_half_employees = add_escort(escorts, index, first_half_employees)
                                    data[extra_cab] = first_half_employees
                                    index += 1
                                    cvd = cvd[cvd['VehicleId'] != extra_cab]
                                else:
                                    last_female = next((emp for emp in reversed(first_half_employees) if emp['gender'].lower() == 'female'), None)
                                    last_male = next((emp for emp in reversed(first_half_employees) if emp['gender'].lower() == 'male'), None)
                                    #print(last_female, last_male)
                                    if last_female and last_male:
                                        distance = haversine(last_male["latitude"], last_male["longitude"], last_female["latitude"], last_female["longitude"])
                                        if distance > 0.5:
                                            first_half_employees = add_escort(escorts, index, first_half_employees)
                                            data[extra_cab] = first_half_employees
                                            index += 1
                                    else:
                                        data[extra_cab] = first_half_employees
                                    cvd = cvd[cvd['VehicleId'] != extra_cab]
        return {'data': data, 'status_code': 200, 'message': "Cab allocations processed successfully"}
    except Exception as e:
        logging.error(f"Error processing cab allocations: {e}")
        return {'error': 'Error processing cab allocations'}

def main_view(request):
    try:
        print("Escort's assigning under processing...")
        histories_to_save = []
        for result_escort_assign in DropingData.objects.all():
            history = Histories(
                booking_id=result_escort_assign.booking_id,
                employee_id=result_escort_assign.employee_id,
                date=result_escort_assign.date,
                in_time=result_escort_assign.in_time,
                out_time=result_escort_assign.out_time,
                employee_name=result_escort_assign.employee_name,
                gender=result_escort_assign.gender,
                address=result_escort_assign.address,
                city=result_escort_assign.city,
                latitude=result_escort_assign.latitude,
                longitude=result_escort_assign.longitude,
                VehicleNumber=result_escort_assign.VehicleNumber,
                SeatCapacity=result_escort_assign.SeatCapacity,
                vehicle_id=result_escort_assign.vehicle_id,
                CumulativeTravelTime=result_escort_assign.CumulativeTravelTime,
                priority_order=result_escort_assign.priority_order,
                escort_id=result_escort_assign.escort_id,
                add_on=datetime.now()
            )
            histories_to_save.append(history)
        Histories.objects.bulk_create(histories_to_save)
        DropingData.objects.all().delete()
        CabAllocation.objects.all().delete()

        response = process_cab_allocations()
        if 'error' not in response:
            data = response['data']
            result_escorts = []
            for vehicle_id, employees in data.items():
                escort_id = None
                for employee in employees:
                    if 'escort_id' in employee:
                        escort_id = employee['escort_id']
                        break
                for employee in employees:
                    if 'id' in employee:
                        if 'booking_id' in employee:
                            result_escort_assign = DropingData(
                                booking_id=employee['booking_id'],
                                employee_id=employee['employee_id'],
                                date=employee['date'],
                                in_time=employee['in_time'],
                                out_time=employee['out_time'],
                                employee_name=employee['employee_name'],
                                gender=employee['gender'].lower(),
                                address=employee['address'],
                                city=employee['city'],
                                latitude=employee['latitude'],
                                longitude=employee['longitude'],
                                VehicleNumber=employee.get('VehicleNumber', None),
                                SeatCapacity=employee.get('SeatCapacity', None),
                                vehicle_id=vehicle_id,
                                CumulativeTravelTime=employee.get('CumulativeTravelTime', None),
                                priority_order=employee.get('priority_order', None),
                                escort_id=escort_id
                            )
                            result_escorts.append(result_escort_assign)
            
            DropingData.objects.bulk_create(result_escorts)
            
            return JsonResponse({'success': "Escorts assigned ...!"}, status=200)
        else:
            return JsonResponse({'Error': "Processing failed...!"}, status=400)
    except Exception as e:
        logging.error(f"Error at escort assigns: {e}")
        return JsonResponse({'error': 'Error saving resultd escort assigns'}, status=500)



