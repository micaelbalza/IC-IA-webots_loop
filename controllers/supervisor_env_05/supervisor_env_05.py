from controller import Supervisor

# Create a supervisor instance
supervisor = Supervisor()

# Define the time step of the simulation
timestep = int(supervisor.getBasicTimeStep())

# Time to add the box in seconds
time_to_add_box1 = 65 #45
time_to_add_box2 = 200 #180
time_to_add_box3 = 350 #330

# Initialize a flag to indicate if the box has been added
box_added1 = False
box_added2 = False
box_added3 = False

# Main loop
while supervisor.step(timestep) != -1:
    # Get the current simulation time
    current_time = supervisor.getTime()

    # Check if the current time has reached the specified time to add the box
    if current_time >= time_to_add_box1 and not box_added1:
        # Create a new node of type 'Solid'
        root = supervisor.getRoot()
        children = root.getField('children')
        
        # Define the box parameters with physical properties
        box_string_1 = (
            'Solid {'
            '  translation -4.5 -1.0 0.1'  # Adjust the position if needed
            '  children ['
            '    Shape {'
            '      appearance Appearance {'
            '        material Material {'
            '          diffuseColor 1 0 0'
            '        }'
            '      }'
            '      geometry Box {'
            '        size 1 1 1'
            '      }'
            '    }'
            '  ]'
            '  boundingObject Box {'
            '    size 1 1 1'
            '  }'
            '  physics Physics {'
            '    mass 10.0'
            '  }'
            '}'
        )
        
        
        # Add the box to the simulation
        children.importMFNodeFromString(-1, box_string_1)
        
        # Set the flag to True to prevent adding multiple boxes
        box_added1 = True

        # Reset simulation physics to update the environment
        supervisor.simulationResetPhysics()
        
        
    if current_time >= time_to_add_box2 and not box_added2:
        # Create a new node of type 'Solid'
        root = supervisor.getRoot()
        children = root.getField('children')
        
        # Define the box parameters with physical properties
        box_string_2 = (
            'Solid {'
            '  translation 0.0 0.5 0.1'  # Adjust the position if needed
            '  children ['
            '    Shape {'
            '      appearance Appearance {'
            '        material Material {'
            '          diffuseColor 1 0 0'
            '        }'
            '      }'
            '      geometry Box {'
            '        size 1 1 1'
            '      }'
            '    }'
            '  ]'
            '  boundingObject Box {'
            '    size 1 1 1'
            '  }'
            '  physics Physics {'
            '    mass 10.0'
            '  }'
            '}'
        )
        
        
        # Add the box to the simulation
        children.importMFNodeFromString(-1, box_string_2)
        
        # Set the flag to True to prevent adding multiple boxes
        box_added2 = True

        # Reset simulation physics to update the environment
        supervisor.simulationResetPhysics()
        
        
    if current_time >= time_to_add_box3 and not box_added3:
        # Create a new node of type 'Solid'
        root = supervisor.getRoot()
        children = root.getField('children')
        
        # Define the box parameters with physical properties
        box_string_3 = (
            'Solid {'
            '  translation 2.5 3.0 0.1'  # Adjust the position if needed
            '  children ['
            '    Shape {'
            '      appearance Appearance {'
            '        material Material {'
            '          diffuseColor 1 0 0'
            '        }'
            '      }'
            '      geometry Box {'
            '        size 1 1 1'
            '      }'
            '    }'
            '  ]'
            '  boundingObject Box {'
            '    size 1 1 1'
            '  }'
            '  physics Physics {'
            '    mass 10.0'
            '  }'
            '}'
        )
        
        
        # Add the box to the simulation
        children.importMFNodeFromString(-1, box_string_3)
        
        # Set the flag to True to prevent adding multiple boxes
        box_added3 = True

        # Reset simulation physics to update the environment
        supervisor.simulationResetPhysics()

# Note: Adjust this part if you need to manually restart specific sensors or perform other updates.
