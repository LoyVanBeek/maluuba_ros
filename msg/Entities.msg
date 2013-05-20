#TYPE        FIELD               USED_IN
string      album                #music   
uint32      rating               #music   
string      playlist             #music       
string      song                 #music   
string      artist               #music   
string      station              #music   
string      genre                #music, theatre   
string      originalTitle        #calendar                   
string      replacementTitle     #calendar                           
string      originalDate         #calendar                   #WAS of type 'time'
string      replacementDate      #calendar                   #WAS of type 'time'
string      originalLocation     #calendar                           
string      replacementLocation  #calendar                           
string      originalTime         #calendar                   #WAS of type 'time'
string      replacementTime      #calendar                   #WAS of type 'time'
string      contactName          #calendar           
string      deleteContactName    #calendar                           
string      duration             #calendar           #WAS of tye 'duration'
TimeRange   dateRange            #calendar, weather, business, theatre           
TimeRange   timeRange            #calendar, transit, business        
string      repeatDaysLength     #calendar                           
string      lengthOfTime         #calendar                   #WAS of tye 'duration'
string[]    repeatDays           #calendar, alarm         
string      meetingTitle         #calendar                   
string      location             #calendar, weather, reminder, transit       
string      date                 #calendar, reminder, alarm, transit   #WAS of type 'time'
Contact[]   contacts             #email, message, call, contact       
string      message              #email,message,reminder   
string      subject              #email   
string      keyword              #email   
string      time                 #weather, reminder, alarm, timer   #WAS of type 'time'
string      leaveLocation        #reminder           
string      destination          #navigation, travel   
string      origin               #navigation   
string      transitType          #navigation, transit       
string      route                #transit   
string      searchTerm           #search, social, business       
uint32      numPeople            #social, business       
string      appName              #application   
string      contactField         #contact           
string      contactFieldValue    #contact               
string      mpaaRating           #theatre       
string      actor                #theatre   
string      theatre              #theatre   
string      title                #theatre   
uint32      numTickets           #theatre       
string      departureTime        #travel           #WAS of type 'time'
string      departureDay         #travel           
string      returnTime           #travel       #WAS of type 'time'
string      returnDay            #travel     
string      departing            #travel       
string      carrier              #travel   
string      sortOrder            #travel       
string      noReturn             #travel       
uint32      child                #travel   
uint32      adult                #travel   
string      senior               #travel   
float32     price                #travel   
string      luxury               #travel  
string      event                #entertainment #Not listed in API docs