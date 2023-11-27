# URDF Files 
---
**tripteron_base**
- This has the basic frames for Tripteron
- In relation to world

**tripteron**
- This has the body attached to the rails/linear sliders
- In relation to world

**tripteronBody**
- This has the body by itself (Not defined in relation to world)
- Does not have the sliders(linear attachment)

**tripteronBodyFull**
- This has the body with the sliders(linear attachment)
- Still redefining sliders/orientation

# Front Legs
---
The front legs and back legs are NOT the same:
- front legs are angled at 45 degrees in the x axis making them stick out more
- They rotate along that slanted axis
- Both sides come back together to the same block
- should come back so that the y position is that same from start to end
# Back Legs
---
- Back legs are angled at 45 degrees in the x axis and 45 degrees in the z them rotate out more
- as a result the resting position is angled higher than in the front legs
- They rotate along that slanted axis
- Both sides are independent of each other
- Both sides are offset so they will not end up in the same y position as when they started

# Dimensions
- Top Legs = 6in in length
- Bottom Legs = 8.48in in length
- Have a raduis of 0.5in 
- platform = 6in x 6in x 3in
