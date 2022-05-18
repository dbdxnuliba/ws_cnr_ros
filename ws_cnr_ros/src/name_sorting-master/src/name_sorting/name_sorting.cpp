#include <name_sorting/name_sorting.h>
#include <ros/console.h>

// ----
namespace name_sorting
{


bool permutationName(  const std::vector<std::string>& order_names,
                       std::vector<std::string>& names,
                       std::vector<double>& position,
                       std::vector<double>& velocity,
                       std::vector<double>& effort,
                       std::stringstream* report)
{
  if (names.size()<order_names.size())
  {
    if(report)
      *report << "The vector of names to be sorted has size " << names.size()
                  << " that is smaller than vector of the sorted names (" << order_names.size() <<")";
    return false;
  }
  if (names.size()!=position.size())
  {
    if(report)
      *report << "Input Mismatch. The vector of names to be sorted has size " << names.size()
                 << " while position size is " << position.size();
    return false;
  }
  if (names.size()!=velocity.size())
  {
    if(report)
      *report << "Input Mismatch. The vector of names to be sorted has size " << names.size()
                << " while velocity size is " << velocity.size();
    return false;
  }
  if (names.size()!=effort.size())
  {
    if(report)
      *report << "Input Mismatch. The vector of names to be sorted has size " << names.size()
                << " while effort size is " << effort.size();
    return false;
  }



  for (unsigned int iOrder=0;iOrder<order_names.size();iOrder++)
  {
    if (names.at(iOrder).compare(order_names.at(iOrder)))
    {
      for (unsigned int iNames=iOrder+1;iNames<names.size();iNames++)
      {
        if (!order_names.at(iOrder).compare(names.at(iNames)))
        {
          std::iter_swap(names.begin()+iOrder,    names.begin()+iNames);
          std::iter_swap(position.begin()+iOrder, position.begin()+iNames);
          std::iter_swap(velocity.begin()+iOrder, velocity.begin()+iNames);
          std::iter_swap(effort.begin()+iOrder,   effort.begin()+iNames);
          break;
        }
        if (iNames==(names.size()-1))
        {
          if(*report)
          {
            *report << "The Joint '" << order_names.at(iOrder) <<"' that is in the vector of the sorted names,"
                    << "is missing in the vector to be sorted.";
            *report << "Sorted Names: <";
            for( size_t i=0;i<order_names.size();i++)
                *report << order_names.at(i) <<",";
            *report << "> vs Names to be ordered: <";
            for( size_t i=0;i<names.size();i++)
              *report << names.at(i) <<",";
            *report <<">";
          }
          return false;
        }
      }
    }
  }
  return true;
}

bool permutationName(  const std::vector<std::string>& order_names,
                       std::vector<std::string>& names,
                       std::vector<double>& position,
                       std::vector<double>& velocity,
                       std::vector<double>& effort,
                       const std::string& whoami)
{
  if (names.size()<order_names.size())
  {
    ROS_ERROR("[ %s ]The vector of names to be sorted has size %zu, that is smaller than vector of the sorted names (%zu)", whoami.c_str(),names.size(),order_names.size());
    return false;
  }
  if (names.size()!=position.size())
  {
    ROS_ERROR("[ %s ]Input Mismatch. The size of the vector of names is %zu, while position size is %zu", whoami.c_str(),names.size(),position.size());
    return false;
  }
  if (names.size()!=velocity.size())
  {
    ROS_ERROR("[ %s ]Input Mismatch. The size of the vector of names is %zu, while velocity size is %zu", whoami.c_str(),names.size(),velocity.size());
    return false;
  }
  if (names.size()!=effort.size())
  {
    ROS_ERROR("[ %s ]Inut Mismatch. The size of the vecotr of the names is %zu, while effort size is %zu", whoami.c_str(),names.size(),effort.size());
    return false;
  }



  for (unsigned int iOrder=0;iOrder<order_names.size();iOrder++)
  {
    if (names.at(iOrder).compare(order_names.at(iOrder)))
    {
      for (unsigned int iNames=iOrder+1;iNames<names.size();iNames++)
      {
        if (!order_names.at(iOrder).compare(names.at(iNames)))
        {
          std::iter_swap(names.begin()+iOrder,    names.begin()+iNames);
          std::iter_swap(position.begin()+iOrder, position.begin()+iNames);
          std::iter_swap(velocity.begin()+iOrder, velocity.begin()+iNames);
          std::iter_swap(effort.begin()+iOrder,   effort.begin()+iNames);
          break;
        }
        if (iNames==(names.size()-1))
        {
          std::stringstream ss;
          ROS_ERROR("[ %s ]The Joint '%s' that is in the vector of the sorted names, is missing in the vector to be sorted.", whoami.c_str(),order_names.at(iOrder).c_str());
          ss << "Sorted Names: <";
    for( size_t i=0;i<order_names.size();i++)
      ss << order_names.at(i) <<",";
          ss << "> vs Names to be ordered: <";
          for( size_t i=0;i<names.size();i++)
      ss << names.at(i) <<",";
          ss <<">";
          ROS_ERROR_STREAM( ss.str() );
          return false;
        }
      }
    }
  }
  return true;
}





bool permutationName(  const std::vector<std::string>& order_names,
                       std::vector<std::string>& names,
                       std::vector<double>& position,
                       std::vector<double>& velocity,
                       std::stringstream* report)
{
  if (names.size()<order_names.size())
  {
    if(report)
      *report << "The vector of names to be sorted has size " << names.size()
                  << " that is smaller than vector of the sorted names (" << order_names.size() <<")";
    return false;
  }
  if (names.size()!=position.size())
  {
    if(report)
      *report << "Input Mismatch. The vector of names to be sorted has size " << names.size()
                 << " while position size is " << position.size();
    return false;
  }
  if (names.size()!=velocity.size())
  {
    if(report)
      *report << "Input Mismatch. The vector of names to be sorted has size " << names.size()
                << " while velocity size is " << velocity.size();
    return false;
  }

  for (unsigned int iOrder=0;iOrder<order_names.size();iOrder++)
  {
    if (names.at(iOrder).compare(order_names.at(iOrder)))
    {
      for (unsigned int iNames=iOrder+1;iNames<names.size();iNames++)
      {
        if (!order_names.at(iOrder).compare(names.at(iNames)))
        {
          std::iter_swap(names.begin()+iOrder,    names.begin()+iNames);
          std::iter_swap(position.begin()+iOrder, position.begin()+iNames);
          std::iter_swap(velocity.begin()+iOrder, velocity.begin()+iNames);
          break;
        }
        if (iNames==(names.size()-1))
        {
          if(*report)
          {
            *report << "The Joint '" << order_names.at(iOrder) <<"' that is in the vector of the sorted names,"
                    << "is missing in the vector to be sorted.";
            *report << "Sorted Names: <";
            for( size_t i=0;i<order_names.size();i++)
                *report << order_names.at(i) <<",";
            *report << "> vs Names to be ordered: <";
            for( size_t i=0;i<names.size();i++)
              *report << names.at(i) <<",";
            *report <<">";
          }
          return false;
        }
      }
    }
  }
  return true;
}

bool permutationName(  const std::vector<std::string>& order_names,
                       std::vector<std::string>& names,
                       std::vector<double>& position,
                       std::vector<double>& velocity,
                       const std::string& whoami)
{
  if (names.size()<order_names.size())
  {
  ROS_ERROR("[ %s ]The vector of names to be sorted has size %zu, that is smaller than vector of the sorted names (%zu)", whoami.c_str(),names.size(),order_names.size());
    return false;
  }
  if (names.size()!=position.size())
  {
    ROS_ERROR("[ %s ]Input Mismatch. The size of the vector of names is %zu, while position size is %zu", whoami.c_str(),names.size(),position.size());
    return false;
  }
  if (names.size()!=velocity.size())
  {
    ROS_ERROR("[ %s ]Input Mismatch. The size of the vector of names is %zu, while velocity size is %zu", whoami.c_str(),names.size(),velocity.size());
    return false;
  }

  for (unsigned int iOrder=0;iOrder<order_names.size();iOrder++)
  {
    if (names.at(iOrder).compare(order_names.at(iOrder)))
    {
      for (unsigned int iNames=iOrder+1;iNames<names.size();iNames++)
      {
        if (!order_names.at(iOrder).compare(names.at(iNames)))
        {
          std::iter_swap(names.begin()+iOrder,    names.begin()+iNames);
          std::iter_swap(position.begin()+iOrder, position.begin()+iNames);
          std::iter_swap(velocity.begin()+iOrder, velocity.begin()+iNames);
          break;
        }
        if (iNames==(names.size()-1))
        {
    std::stringstream ss;
          ROS_ERROR("[ %s ]The Joint '%s' that is in the vector of the sorted names, is missing in the vector to be sorted.", whoami.c_str(),order_names.at(iOrder).c_str());
          ss << "Sorted Names: <";
    for( size_t i=0;i<order_names.size();i++)
      ss << order_names.at(i) <<",";
          ss << "> vs Names to be ordered: <";
          for( size_t i=0;i<names.size();i++)
      ss << names.at(i) <<",";
          ss <<">";
          ROS_ERROR_STREAM( ss.str() );

          return false;
        }
      }
    }
  }
  return true;
}





bool permutationName(  const std::vector<std::string>& order_names,
                       std::vector<std::string>& names,
                       std::vector<double>& position,
                       std::stringstream* report)
{
  if (names.size()<order_names.size())
  {
    if(report)
      *report << "The vector of names to be sorted has size " << names.size()
                  << " that is smaller than vector of the sorted names (" << order_names.size() <<")";
    return false;
  }
  if (names.size()!=position.size())
  {
    if(report)
      *report << "Input Mismatch. The vector of names to be sorted has size " << names.size()
                 << " while position size is " << position.size();
    return false;
  }

  for (unsigned int iOrder=0;iOrder<order_names.size();iOrder++)
  {
    if (names.at(iOrder).compare(order_names.at(iOrder)))
    {
      for (unsigned int iNames=iOrder+1;iNames<names.size();iNames++)
      {
        if (!order_names.at(iOrder).compare(names.at(iNames)))
        {
          std::iter_swap(names.begin()+iOrder,    names.begin()+iNames);
          std::iter_swap(position.begin()+iOrder, position.begin()+iNames);
          break;
        }
        if (iNames==(names.size()-1))
        {
          if(*report)
          {
            *report << "The Joint '" << order_names.at(iOrder) <<"' that is in the vector of the sorted names,"
                    << "is missing in the vector to be sorted.";
            *report << "Sorted Names: <";
            for( size_t i=0;i<order_names.size();i++)
                *report << order_names.at(i) <<",";
            *report << "> vs Names to be ordered: <";
            for( size_t i=0;i<names.size();i++)
              *report << names.at(i) <<",";
            *report <<">";
          }
          return false;
        }
      }
    }
  }
  return true;
}


bool permutationName(  const std::vector<std::string>& order_names,
                       std::vector<std::string>& names,
                       std::vector<double>& position,
                       const std::string& whoami)
{
  if (names.size()<order_names.size())
  {
    ROS_ERROR("[ %s ] The vector of names to be sorted has size %zu, that is smaller than vector of the sorted names (%zu)", whoami.c_str(), names.size(), order_names.size());
    return false;
  }
  if (names.size()!=position.size())
  {
    ROS_ERROR("[ %s ] Input Mismatch. The size of the vector of names is %zu, while position size is %zu", whoami.c_str(),names.size(),position.size());
    return false;
  }

  for (unsigned int iOrder=0;iOrder<order_names.size();iOrder++)
  {
    if (names.at(iOrder).compare(order_names.at(iOrder)))
    {
      for (unsigned int iNames=iOrder+1;iNames<names.size();iNames++)
      {
        if (!order_names.at(iOrder).compare(names.at(iNames)))
        {
          std::iter_swap(names.begin()+iOrder,    names.begin()+iNames);
          std::iter_swap(position.begin()+iOrder, position.begin()+iNames);
          break;
        }
        if (iNames==(names.size()-1))
        {
          std::stringstream ss;
          ROS_ERROR("[ %s ]The Joint '%s' that is in the vector of the sorted names, is missing in the vector to be sorted.",whoami.c_str(),order_names.at(iOrder).c_str());
          ss << "Sorted Names: <";
          for( size_t i=0;i<order_names.size();i++)
            ss << order_names.at(i) <<",";
          ss << "> vs Names to be ordered: <";
          for( size_t i=0;i<names.size();i++)
            ss << names.at(i) <<",";
          ss <<">";
          ROS_ERROR_STREAM( ss.str() );
          return false;
        }
      }
    }
  }
  return true;
}

  
}

