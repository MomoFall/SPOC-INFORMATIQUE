cmake_minimum_required(VERSION 3.8)
project(spoc_interfaces)

# Générateur ROS 2 : il transforme srv/LireCapteur.srv en code C/C++/Python.
find_package(rosidl_default_generators REQUIRED)

# Déclare les interfaces du package. Tout nouveau .srv devra être ajouté
# dans cette liste, sinon il ne sera pas généré.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LireCapteur.srv"
)

ament_package()
