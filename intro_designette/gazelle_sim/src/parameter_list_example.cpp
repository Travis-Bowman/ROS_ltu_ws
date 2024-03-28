// Sum a list of doubles from the parameter server
std::vector<double> my_double_list;
double sum = 0;
nh.getParam("my_double_list", my_double_list);
for(unsigned i=0; i < my_double_list.size(); i++) {
  sum += my_double_list[i];
}
