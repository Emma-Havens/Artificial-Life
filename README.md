# Artificial-Life

Robot is supposed to be evolved to climb up the stairs. The fitness function considers how far the robot traveled up the stairs (x and z directions)
and not putting its head into the ground (head touch sensor not being -1), all equally weighted and multiplied together.

Originally I also tried to make it advantageous for it to lift its feet off the ground (feet sensors being -1 for long periods of time) but that just
made it curl up on the ground like a dead spider, apparently it being more advantageous to do that than not having its head off the ground or moving. 
First I tried making it only want to make its front two legs lift off the ground in hope that it would rear upwards, but doing so didn't improve its 
climbing capabilities. Then I tried all 4 legs, resulting in dead spiders.

Project can easily be run a reproduced by running 'python search.py'
