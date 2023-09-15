**Installation**
follow the steps below to install the project
1. Install Python 3.6 or higher
2. Install pip
3. Install the following packages:
   - django 2.2.6 or higher
   - django-rest-framework
   - WebSockets
   - ROSBridge
4. Clone the repository
5. Run the following commands:
   - python manage.py makemigrations
   - python manage.py migrate
   - python manage.py createsuperuser
   - python manage.py runserver
  
6. Go to http://localhost:8000/index.html to access the robot control user interface

**URLS**
1. Go to http://localhost:8000/index.html to access the robot control user interface
2. Go to http://localhost:8000/admin to access the admin interface
3. Go to http://localhost:8000/wizard/setup to access the setup wizard
4. The following are the API endpoints:
   - /reqpub -> GET: loads the main user interface, POST: recieves movement commands and sends them to the robot
   - /reqcli -> GET: passes the user-selected emotion's data to the robot , POST: recieves the robot's response
   - /reqemo -> GET: sends back the video url of the selected sound, or the sound url of the selected face (video)
   - /reqip -> GET: sends back the IP address of the robot (fetchs server local ip address)

   
**File Structure**
1. interface_backend/ contains the main project
2. core/ contains the main app -> handles the main user interface and between-device communication (a.k.a. the android app)
3. soundHandler/ contains the soundHandler app -> handles sound related models
4. serialHandler/ contains the serialHandler app -> handles serial communication related models
5. static/ contains the static files (css, js, images, etc.)
6. templates/ contains the html templates
7. db.sqlite3 is the database file
8. manage.py is the main file for running the server
9. requirements.txt contains the required packages for the project

**API Endpoints**
1. core/views.py contains the main API endpoints
2. soundHandler/views.py contains the API endpoints for the soundHandler app
3. serialHandler/views.py contains the API endpoints for the serialHandler app

**Models**
1. core/models.py contains the main models:
 - EmotionModel
    - face -> name of the emotion
    - face_video_url -> url of the video for the emotion
    - interface_button_emoji -> emoji for the emotion (used in the user interface)
  
  - Motor Control models:
    - dynatype -> type of the dynamixel motor
    - movement -> True if the motor must move, False otherwise
    - dir -> direction of the movement (0 for (up - right) or 1 for (down - left))
    - pos_up -> Neck pitch turn
    - pos_right -> Neck yaw turn
    - right_hand -> Right hand final position
    - left_hand -> Left hand final position
    - speed -> Speed of the movement
    - theta -> Angle of the movement
    - yaw   -> Yaw of the movement
    - 
2. soundHandler/models.py contains the models for the soundHandler app
   - Song:
        - title -> title of the sound file
        - description -> description of the sound file
        - audio_file -> the sound file (optional)
        - audio_link -> the link to the sound file (optional)
        - duration -> the duration of the sound file


**Deployment**
You need a web server to run the project. I used nginx and gunicorn. You can use apache or any other web server. To have direct access to the database, you need to be a superuser.
To create a superuser, run the following command: python manage.py createsuperuser
Now you can access the database at /admin with admin privileges. (e.g. modify model objects, add new users, etc.)

To run the project from scratch, run the following command: gunicorn --bind host:port --workers 3 --threads 2 --timeout 120 --log-level debug --log-file log.txt --access-logfile access_log.txt --error-logfile error_log.txt --capture-output --enable-stdio-inheritance --daemon --pid pid.txt --pythonpath [path/to/project] [project_name].wsgi:application

To use the built-in django web-server, run the following command: python manage.py runserver. 
However, the server must have automaticly been served by the web server (e.g. nginx) in production environment (e.g. Jetson Nano) and gunicorn service & socket in the background on each reboot.

If not, check nginx status to make sure it is active and then check for errors in the log file: /var/log/nginx/error.log and /var/log/nginx/access.log. If you are using a different web server, check its log file.

Also this could have happened because of failed gunicorn service. Check the status of gunicorn service with the following command: sudo systemctl status gunicorn and check gunicorn configuration file: /etc/systemd/system/gunicorn.service and gunicorn.socket configuration file: /etc/systemd/system/gunicorn.socket.

**NOTE** *Make sure you have allowed the ports in use (e.g. 1935, 5353, 8080, etc.) in the firewall. To check the status of the firewall, run the following command: sudo ufw status. To allow a port, run the following command: sudo ufw allow [port number]. To allow a range of ports, run the following command: sudo ufw allow [port number]:[port number]*

**Installation**
1. Install nginx
2. Install gunicorn
3. Install supervisor
4. Create a new user for the project
5. Create a new directory for the project
6. Clone the project into the new directory
7. Create a new virtual environment for the project
8. Install the requirements for the project
9. Create a new configuration file for the project in /etc/nginx/sites-available/
10. Create a new configuration file for the project in /etc/supervisor/conf.d/
11. Create a new configuration file for the gunicorn service in /etc/systemd/system/
12. Setup the gunicorn service
13. Setup nginx

14. To setup the gunicorn service, use the following commands:
    - sudo systemctl start gunicorn
    - sudo systemctl enable gunicorn
    - sudo systemctl status gunicorn
    - sudo systemctl stop gunicorn
    - sudo systemctl restart gunicorn
    - sudo systemctl daemon-reload
    - sudo systemctl reload gunicorn
    - sudo systemctl disable gunicorn
    - 
15. Use the following commands for nginx:
    - sudo systemctl daemon-reload
    - sudo systemctl restart nginx
    - sudo systemctl status nginx
    - sudo systemctl stop nginx
    - sudo systemctl start nginx
    - sudo systemctl reload nginx
    - sudo systemctl disable nginx
    - sudo systemctl enable nginx


**Notes**
*If you want to be able to access the user-interface from other devices in the same network, you should use the same url but replace the IP address with the IP address of the main server (e.g. Jetson). You can also use a domain name instead of the IP address.*

*Add the following line to nginx sites-enabled/[my website's name] configuration file because your media files will be ignored by the browser if they are served with the default mimetype of text/plain : "include /etc/nginx/mime.types;"*

*If requirements.txt does not exist, run "pipreqs [path/to/ project]" to generate requirements.txt file for your project and install the requirements with pip install -r requirements.txt*

*The user-interface must be accessible from browser with the following address: http://[Jetson|Computer's IP address]:[port number(default:5353)]/index.html*

# Additions to the project****************************************************************

If you want to assign a static IP address to the Jetson Nano, you can use the following command: sudo nmtui.
To check the IP address of the Jetson Nano, run the following command: hostname -I.
To check the IP address of the computer, run the following command: ipconfig.

If you want to deploy the project or assign a purchased domain name to the project, you must change the server_name in the nginx configuration file from localhost to the domain name. Be sure to configure the DNS settings of the domain name to point to the IP address of the server using cloudflare or any other DNS service provider.


**Daemon**
To run the project as a daemon, you can use the following command: gunicorn --bind host:port --workers 3 --threads 2 --timeout 120 --log-level debug --log-file log.txt --access-logfile access_log.txt --error-logfile error_log.txt --capture-output --enable-stdio-inheritance --daemon --pid pid.txt --pythonpath [path/to/project] [project_name].wsgi:application

**Database**
type of the database: sqlite3
location of the database: [path/to/project]/db.sqlite3
To access the database, you need to be a superuser. To create a superuser, run the following command: python manage.py createsuperuser
Now you can access the database at /admin with admin privileges. (e.g. modify model objects, add new users, etc.)

**Serializers**
All of the following are EmotionModel serializers:
- EmotionModelSerializer
- HooshangDynaSerializerHead 
- HooshangDynaSerializerHands
- HooshangDynaSerializer

**URLs**
1. core/urls.py contains the main urls
   - / -> the main user interface
   - /admin -> the admin interface
   - 