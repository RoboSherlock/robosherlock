/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */


#include "uima/api.hpp"
using namespace uima;
using namespace std;
class MeetingAnnotator : public Annotator {
private:
  //config param window size
  int mWindowSize;

  //Input types
  Type roomNumberT;
  Type dateT;
  Type timeT;

  //output type and its features
  Type meetingT;
  Feature roomF;
  Feature dateF;
  Feature startTimeF;
  Feature endTimeF;


public:

  MeetingAnnotator(void)
  {
    cout << "MeetingAnnotator: Constructor" << endl;
  }

  ~MeetingAnnotator(void)
  {
    cout << "MeetingAnnotator: Destructor" << endl;
  }

  /** */
  TyErrorId initialize(AnnotatorContext & rclAnnotatorContext)
  {
    cout << "MeetingAnnotator: initialize()" << endl;

    if (!rclAnnotatorContext.isParameterDefined("WindowSize") ||
	rclAnnotatorContext.extractValue("WindowSize", mWindowSize) != UIMA_ERR_NONE) {
      /* log the error condition */
      rclAnnotatorContext.getLogger().logError("Required configuration parameter \"WindowSize\" not found in component descriptor");
      cout << "initialize() - Error. See logfile." << endl;
      return UIMA_ERR_USER_ANNOTATOR_COULD_NOT_INIT;
    }

	if (mWindowSize < 50) {
      rclAnnotatorContext.getLogger().logWarning("WindowSize may be too narrow");
    }

    /* log the configuration parameter setting */
	std::stringstream ss;
	std::string str;
	ss << mWindowSize;
	ss >> str;
    rclAnnotatorContext.getLogger().logMessage("WindowSize = '" + str + "'");

    cout << "initialize() .. mWindowSize.getBuffer: "
	 << mWindowSize << endl;

    return (TyErrorId)UIMA_ERR_NONE;
  }

  /** */
  TyErrorId typeSystemInit(TypeSystem const & crTypeSystem) {
    cout << " typeSystemInit()" << endl;

    //input types
    roomNumberT =  crTypeSystem.getType("org.apache.uima.tutorial.RoomNumber");
    dateT =  crTypeSystem.getType("org.apache.uima.tutorial.DateAnnot");
    timeT =  crTypeSystem.getType("org.apache.uima.tutorial.TimeAnnot");

    //output types and features
    meetingT =  crTypeSystem.getType("org.apache.uima.tutorial.Meeting");

    if ( !(meetingT.isValid() && roomNumberT.isValid() && dateT.isValid() && timeT.isValid()) ) {
        getAnnotatorContext().getLogger().logError("typeSystemInit() Error getting Type objects");

	cout << "typeSystemInit() Error getting Type object" << endl;
	 return (TyErrorId) UIMA_ERR_RESMGR_INVALID_RESOURCE;
    }

    //get features
    roomF = meetingT.getFeatureByBaseName("room");
    dateF = meetingT.getFeatureByBaseName("date");
    startTimeF = meetingT.getFeatureByBaseName("startTime");
    endTimeF = meetingT.getFeatureByBaseName("endTime");

    if ( !(roomF.isValid() && dateF.isValid() && startTimeF.isValid() && endTimeF.isValid() ) ) {
	  getAnnotatorContext().getLogger().logError("typeSystemInit() Error getting Feature objects");
	  cout << "typeSystemInit() Error getting Feature  object" << endl;
	 return (TyErrorId) UIMA_ERR_RESMGR_INVALID_RESOURCE;
  }

    return (TyErrorId)UIMA_ERR_NONE;
  }

  /** */
  TyErrorId destroy()
  {
    cout << "MeetingAnnotator: destroy()" << endl;
    return (TyErrorId)UIMA_ERR_NONE;
  }

  /** */
  TyErrorId process(CAS & tcas, ResultSpecification const & crResultSpecification)
  {
    cout << "MeetingAnnotator: process()" << endl;

    //get annotation indexes
    FSIndexRepository & indexes = tcas.getIndexRepository();

    ANIndex roomNumberIndex = tcas.getAnnotationIndex(roomNumberT);
    ANIndex dateIndex = tcas.getAnnotationIndex(dateT);
    ANIndex timeIndex = tcas.getAnnotationIndex(timeT);

    //store end position of last meeting we identified, to prevent multiple
    //annotations over same span
    int lastMeetingEnd = -1;

    //iterate over all combinations
    ANIterator roomNumberIter = roomNumberIndex.iterator();
    roomNumberIter.moveToFirst();
    while (roomNumberIter.isValid()) {
      AnnotationFS room = roomNumberIter.get();

      ANIterator dateIter = dateIndex.iterator();
      dateIter.moveToFirst();

      while (dateIter.isValid()) {
        AnnotationFS date = dateIter.get();
        ANIterator time1Iter = timeIndex.iterator();
        time1Iter.moveToFirst();

        while (time1Iter.isValid()) {
          AnnotationFS time1 = time1Iter.get();
          ANIterator time2Iter = timeIndex.iterator();
          time2Iter.moveToFirst();

          while (time2Iter.isValid()) {
            AnnotationFS time2 = time2Iter.get();

            //times must be different annotations
            if (time1 != time2) {
              //compute the begin and end of the span
              int minBegin = min(
				  min(time1.getBeginPosition(), time2.getBeginPosition()),
				  min(date.getBeginPosition(), room.getBeginPosition()));

			  int maxEnd = max(
		  		  max(time1.getEndPosition(), time2.getEndPosition()),
		  		  max(date.getEndPosition(), room.getEndPosition()));

			  //span must be smaller than the window size?
			  if (maxEnd - minBegin < mWindowSize) {
				//span must not overlap the last annotation we made
				if (minBegin > lastMeetingEnd) {
				  //annotate
				  //Create Meeting FS and set the begin and end features
				  AnnotationFS mtg = tcas.createAnnotation(meetingT, minBegin, maxEnd);
				  //Set other four features
				  mtg.setFSValue(roomF, room);
				  mtg.setFSValue(dateF, date);
				  mtg.setFSValue(startTimeF, time1);
				  mtg.setFSValue(endTimeF, time2);

				  //add annotation to the index
				  indexes.addFS(mtg);
				  lastMeetingEnd = maxEnd;
				}
			  }
			}
			time2Iter.moveToNext();
			}
			time1Iter.moveToNext();
		  }
		  dateIter.moveToNext();
		}
		roomNumberIter.moveToNext();
	}
	return (TyErrorId)UIMA_ERR_NONE;
  }
};


// This macro exports an entry point that is used to create the annotator.

MAKE_AE(MeetingAnnotator);

