import time
import os

class Utilities():

    def get_response_time_msg(start_time, message='Response Sent: Path Found........'):
        """Log the time taken to respond to a request."""
        end_time = time.time() 
        return f'{message}....... (Response time: {str(end_time-start_time)} seconds)\n'


    def create_log_file(log_file_path, occupancy_grid, start_pose, end_pose, path):
        '''Create a log file for the planner server.'''
        directory = log_file_path
        os.makedirs(directory, exist_ok=True)
        new_log_index = len(os.listdir(directory)) + 1
        
        # Define the log name with the new index
        log_name = f"planner_server{new_log_index}.log"
        
        # Create the log file and write initial content
        with open(os.path.join(directory, log_name), "w") as file:
            file.write('Planner Server Log File\n\n')
            file.write('Request:\n')
            file.write(f'Start Pose: {start_pose}\n')
            file.write(f'End Pose: {end_pose}\n')
            file.write(f'Occupancy Grid: \n')
            for row in occupancy_grid:
                file.write(' '.join(str(cell) for cell in row) + '\n')
            file.write('\n')
            file.write('Response:\n')
            file.write('Path Poses:\n')
            for pose in path:
                file.write(f'{pose}\n')
        
        print(f"New log file created: {os.path.join(directory, log_name)}")
        print(f"Total number of log files: {new_log_index}")