#!/bin/bash
#---------------------------------------------------
#  Script: clean.sh
#  Author: Michael DeFilippo, adapted from moos-ivp clean.sh (Michael Benjamin)
#    Date: Jan 2023
#---------------------------------------------------
#  Part 1: Declare global var defaults                                                                  
#----------------------------------------------------------                                             
VERBOSE=""                                                                                              
                                                                                                        
#-------------------------------------------------------                                                
#  Part 1: Check for and handle command-line arguments                                                  
#-------------------------------------------------------                                                
for ARGI; do                                                                                            
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then                                               
        echo "clean.sh [SWITCHES]        "                                                              
        echo "   --verbose, -v           "                                                              
        echo "   --help, -h              "                                                              
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then                                          
        VERBOSE="-v"                                                                                    
    else                                                                                                
        echo "clean.sh: Bad Arg:" $ARGI                                                                 
        exit 1                                                                                          
    fi                                                                                                  
done                                                                                                    
                                                                                                        
#-------------------------------------------------------                                                
#  Part 2: Do the cleaning!                                                                             
#-------------------------------------------------------                                                
if [ "${VERBOSE}" = "-v" ]; then                                                                        
    echo "Cleaning: $PWD"                                                                               
fi                                                                                                      

# clean emacs leftovers 
rm -f   $VERBOSE   *~ .\#* \#*\# 
find . -name '*~'  -print -exec rm -rfv {} \;                                                           
find . -name '#*'  -print -exec rm -rfv {} \; 

                                               
