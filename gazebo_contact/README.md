# Contact

Run the contact.world using Gazebo:

$ gazebo contact.world

In a separate terminal list the topics published by Gazebo

$ gz topic -l

Print the value of the contact sensors to the screen:

$ gz topic -e /gazebo/default/box/link/my_contact
