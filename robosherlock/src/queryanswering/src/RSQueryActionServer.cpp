#include <rs/queryanswering/RSQueryActionServer.h>
#include <rs/flowcontrol/RSProcessManager.h>

/*
 * Constructor
 */

RSQueryActionServer::RSQueryActionServer(ros::NodeHandle n, RSProcessManager* rsMngr)
{
  this->run(n, rsMngr);
}

/*
 * Server
 */
RSQueryActionServer::~RSQueryActionServer()
{
}

/*
 * sendFeedback: This function publishes from time to time feedback about the query execution
 */

void RSQueryActionServer::sendFeedback(std::basic_string<char> status)
{
  robosherlock_msgs::RSQueryFeedback feedback;
  feedback.status.swap(status);
  this->server->publishFeedback(feedback);
}

/*
 * Convert ObjectDesignator to Json String
 */

std::string* RSQueryActionServer::objectDesignator_to_jsonString(const robosherlock_msgs::RSQueryGoalConstPtr objPtr)
{
  // initialize query
  std::string query = "";
  bool comma = false;
  // adding header to query
  query.append("{\"detect\":{");

  // adding color filter to query
  if (comma && objPtr->obj.color.size() > 0)
    query.append(",");
  for (int i = 0; i < objPtr->obj.color.size(); i++)
  {
    comma = true;
    query.append("\"color\":");
    query.append("\"");
    query.append(objPtr->obj.color[i]);
    query.append("\"");
    if (i != (objPtr->obj.color.size() - 1))
      query.append(",");
  }

  // adding shape filter to query
  if (comma && objPtr->obj.shape.size() > 0)
    query.append(",");
  for (int i = 0; i < objPtr->obj.shape.size(); i++)
  {
    comma = true;
    query.append("\"shape\":");
    query.append("\"");
    query.append(objPtr->obj.shape[i]);
    query.append("\"");
    if (i != (objPtr->obj.shape.size() - 1))
      query.append(",");
  }

  // adding poseSource filter to query
  if (comma && objPtr->obj.poseSource.size() > 0)
    query.append(",");
  for (int i = 0; i < objPtr->obj.poseSource.size(); i++)
  {
    comma = true;
    query.append("\"cad-model\":");
    query.append("\"");
    query.append(objPtr->obj.poseSource[i]);
    query.append("\"");
    if (i != (objPtr->obj.poseSource.size() - 1))
      query.append(",");
  }

  // adding size filter to query
  if (comma && objPtr->obj.size.size() > 0)
    query.append(",");
  if (objPtr->obj.size.size() > 0)
  {
    comma = true;
    query.append("\"size\":");
    query.append("\"");
    query.append(objPtr->obj.size);
    query.append("\"");
  }

  // adding location filter to query
  if (comma && objPtr->obj.location.size() > 0)
    query.append(",");
  if (objPtr->obj.location.size() > 0)
  {
    comma = true;
    query.append("\"location\":");
    query.append("\"");
    query.append(objPtr->obj.location);
    query.append("\"");
  }

  // adding type filter to query
  if (comma && objPtr->obj.type.size() > 0)
    query.append(",");
  if (objPtr->obj.type.size() > 0)
  {
    comma = true;
    query.append("\"type\":");
    query.append("\"");
    query.append(objPtr->obj.type);
    query.append("\"");
  }

  // adding uid filter to query
  if (comma && objPtr->obj.uid.size() > 0)
    query.append(",");
  if (objPtr->obj.uid.size() > 0)
  {
    comma = true;
    query.append("\"uid\":");
    query.append("\"");
    query.append(objPtr->obj.uid);
    query.append("\"");
  }

  // close query
  query.append("}}");

  return new std::string(query);
}

/*
 * Convert Json String to ObjectDesignator
 */

robosherlock_msgs::ObjectDesignator* RSQueryActionServer::jsonString_to_objectDesignator(std::string jsonStr)
{
  // a json document to parse the jsonStr
  rapidjson::Document answer;

  // an object designator
  robosherlock_msgs::ObjectDesignator* objPtr = new robosherlock_msgs::ObjectDesignator();

  // parse a json answer
  answer.Parse(jsonStr);
  // object uid
  if (answer.HasMember("id"))
    objPtr->uid = answer["id"].GetString();
  // object class a.k.a. type
  if (answer.HasMember("rs.annotation.Classification"))
  {
    if (answer["rs.annotation.Classification"].IsArray())
      // only consider the first class
      for (int i = 0; i < answer["rs.annotation.Classification"].GetArray().Size(); i++)
        objPtr->type = answer["rs.annotation.Classification"][i]["classname"].GetString();
    else
    {
      objPtr->type = answer["rs.annotation.Classification"]["classname"].GetString();
    }
  }
  else
  {
    if (answer.HasMember("rs.annotation.Detection"))
    {
      if (answer["rs.annotation.Detection"].IsArray())
        // only consider the first class
        for (int i = 0; i < answer["rs.annotation.Detection"].GetArray().Size(); i++)
          objPtr->type = answer["rs.annotation.Detection"][i]["name"].GetString();
      else
      {
        objPtr->type = answer["rs.annotation.Detection"]["name"].GetString();
      }
    }
  }

  // object's semantic size a.k.a. size
  if (answer.HasMember("rs.annotation.SemanticSize"))
  {
    if (answer["rs.annotation.SemanticSize"].IsArray())
      // only consider the first size
      for (int i = 0; i < answer["rs.annotation.SemanticSize"].GetArray().Size(); i++)
        objPtr->size = answer["rs.annotation.SemanticSize"][i]["size"].GetString();
    else
    {
      objPtr->size = answer["rs.annotation.SemanticSize"]["size"].GetString();
    }
  }

  // object's location a.k.a. location
  if (answer.HasMember("rs.annotation.SemanticLocation"))
  {
    if (answer["rs.annotation.SemanticLocation"].IsArray())
      // only consider the first location
      for (int i = 0; i < answer["rs.annotation.SemanticLocation"].GetArray().Size(); i++)
        objPtr->location = answer["rs.annotation.SemanticLocation"][i]["location"].GetString();
    else
    {
      objPtr->location = answer["rs.annotation.SemanticLocation"]["location"].GetString();
    }
  }

  // object's color a.k.a. color
  if (answer.HasMember("rs.annotation.SemanticColor"))
  {
    if (answer["rs.annotation.SemanticColor"].IsArray())
      for (int i = 0; i < answer["rs.annotation.SemanticColor"].GetArray().Size(); i++)
        objPtr->color.push_back(answer["rs.annotation.SemanticColor"][i]["color"].GetString());
    else
    {
      objPtr->color.push_back(answer["rs.annotation.SemanticColor"]["color"].GetString());
    }
  }
  // object's shape a.k.a. shape
  if (answer.HasMember("rs.annotation.Shape"))
  {
    if (answer["rs.annotation.Shape"].IsArray())
      for (int i = 0; i < answer["rs.annotation.Shape"].GetArray().Size(); i++)
        objPtr->shape.push_back(answer["rs.annotation.Shape"][i]["shape"].GetString());
    else
    {
      objPtr->shape.push_back(answer["rs.annotation.Shape"]["shape"].GetString());
    }
  }

  // object's 6D-pose a.k.a. poseStamped
  if (answer.HasMember("rs.annotation.PoseAnnotation"))
  {
    if (answer["rs.annotation.PoseAnnotation"].IsArray())
    {
      // resize pose array
      objPtr->pose.resize(answer["rs.annotation.PoseAnnotation"].GetArray().Size());
      for (int i = 0; i < answer["rs.annotation.PoseAnnotation"].GetArray().Size(); i++)
      {
        if (answer["rs.annotation.PoseAnnotation"][i].HasMember("world"))
        {
          if (answer["rs.annotation.PoseAnnotation"][i]["world"].HasMember("rs.tf.StampedPose"))
          {
            // poseStamped's head
            if (answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"].HasMember("timestamp"))
              objPtr->pose[i].header.seq =
                  answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"]["timestamp"].GetInt64();
            if (answer.HasMember("timestamp"))
              objPtr->pose[i].header.stamp = ros::Time(answer["timestamp"].GetDouble());

            if (answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"].HasMember("frame"))
              objPtr->pose[i].header.frame_id =
                  answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"]["frame"].GetString();

            // poseStamped's orientation
            if (answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"].HasMember("rotation"))
            {
              objPtr->pose[i].pose.orientation.x =
                  answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"]["rotation"]
                      .GetArray()[0]
                      .GetDouble();
              objPtr->pose[i].pose.orientation.y =
                  answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"]["rotation"]
                      .GetArray()[1]
                      .GetDouble();
              objPtr->pose[i].pose.orientation.z =
                  answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"]["rotation"]
                      .GetArray()[2]
                      .GetDouble();
              objPtr->pose[i].pose.orientation.w =
                  answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"]["rotation"]
                      .GetArray()[3]
                      .GetDouble();
            }

            // poseStamped's position
            if (answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"].HasMember("translation"))
            {
              objPtr->pose[i].pose.position.x =
                  answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"]["translation"]
                      .GetArray()[0]
                      .GetDouble();
              objPtr->pose[i].pose.position.y =
                  answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"]["translation"]
                      .GetArray()[1]
                      .GetDouble();
              objPtr->pose[i].pose.position.z =
                  answer["rs.annotation.PoseAnnotation"][i]["world"]["rs.tf.StampedPose"]["translation"]
                      .GetArray()[2]
                      .GetDouble();
            }
          }
        }
        if (answer["rs.annotation.PoseAnnotation"][i].HasMember("source"))
          objPtr->poseSource.push_back(answer["rs.annotation.PoseAnnotation"][i]["source"].GetString());
      }
    }
    else
    {
      // resize pose array
      objPtr->pose.resize(1);

      if (answer["rs.annotation.PoseAnnotation"].HasMember("world"))
      {
        if (answer["rs.annotation.PoseAnnotation"]["world"].HasMember("rs.tf.StampedPose"))
        {
          // poseStamped's head
          if (answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"].HasMember("timestamp"))
            objPtr->pose[0].header.seq =
                answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["timestamp"].GetInt64();
          if (answer.HasMember("timestamp"))
            objPtr->pose[0].header.stamp = ros::Time(answer["timestamp"].GetDouble());
          if (answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"].HasMember("frame"))
            objPtr->pose[0].header.frame_id =
                answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["frame"].GetString();

          // poseStamped's orientation
          if (answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"].HasMember("rotation"))
          {
            objPtr->pose[0].pose.orientation.x =
                answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["rotation"]
                    .GetArray()[0]
                    .GetDouble();
            objPtr->pose[0].pose.orientation.y =
                answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["rotation"]
                    .GetArray()[1]
                    .GetDouble();
            objPtr->pose[0].pose.orientation.z =
                answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["rotation"]
                    .GetArray()[2]
                    .GetDouble();
            objPtr->pose[0].pose.orientation.w =
                answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["rotation"]
                    .GetArray()[3]
                    .GetDouble();
          }

          // poseStamped's position
          if (answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"].HasMember("translation"))
          {
            objPtr->pose[0].pose.position.x =
                answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["translation"]
                    .GetArray()[0]
                    .GetDouble();
            objPtr->pose[0].pose.position.y =
                answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["translation"]
                    .GetArray()[1]
                    .GetDouble();
            objPtr->pose[0].pose.position.z =
                answer["rs.annotation.PoseAnnotation"]["world"]["rs.tf.StampedPose"]["translation"]
                    .GetArray()[2]
                    .GetDouble();
          }
        }
      }
      if (answer["rs.annotation.PoseAnnotation"].HasMember("source"))
        objPtr->poseSource.push_back(answer["rs.annotation.PoseAnnotation"]["source"].GetString());
    }
  }

  return objPtr;
}

/*
 * Execute: Core function of the action server
 */
void RSQueryActionServer::executeQuery(const robosherlock_msgs::RSQueryGoalConstPtr& goal,
                                       RSQueryActionServer* RSQueryActionServer, RSProcessManager* rsMngr)
{
  // set ProcessManager
  RSQueryActionServer->rsProcessManager = rsMngr;
  // shared result pointer
  robosherlock_msgs::RSQueryResult result;
  // publish a feedback about query activation
  std::basic_string<char> status = "Query received: 0% completion";
  RSQueryActionServer->sendFeedback(status);

  status = "Starting query processing ...";
  RSQueryActionServer->sendFeedback(status);

  std::string goalString = *RSQueryActionServer->objectDesignator_to_jsonString(goal);
  std::vector<std::string> resString = {};
  // process query and publish the exit status and query result
  if (rsMngr != nullptr)
  {
    if (!(rsMngr->handleQuery(goalString, resString)))
    {
      // empty answer
      result.res.empty();
      status = "Error: Aborted Query - Check query formatting!!!";
      RSQueryActionServer->sendFeedback(status);
      RSQueryActionServer->server->setAborted(result);
    }
    else
    {
      // query answer and status
      status = "Result Processing: 75% completion";
      RSQueryActionServer->sendFeedback(status);
      for (int i = 0; i < resString.size(); i++)
      {
        result.res.push_back(*(RSQueryActionServer->jsonString_to_objectDesignator(resString[i])));
      }
      status = "Succeded Query: 100% completion";
      RSQueryActionServer->sendFeedback(status);
      RSQueryActionServer->server->setSucceeded(result);
    }
  }
  else
  {
    status = "Error: No perception system started!!!";
    RSQueryActionServer->sendFeedback(status);
    // empty answer
    result.res.empty();
    RSQueryActionServer->server->setAborted(result);
  }
}

int RSQueryActionServer::run(ros::NodeHandle n, RSProcessManager* rsMngr)
{
  ROS_INFO("Server was started!");
  this->server = new Server(n, std::string("/rs_query_action_").append(getenv("USER")),
                            boost::bind(&(RSQueryActionServer::executeQuery), _1, this, rsMngr), false);
  this->server->start();
  return 0;
}
