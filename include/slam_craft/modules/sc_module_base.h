/*
* @Descripttion: 
* @Author: Meng Kai
* @version: 
* @Date: 2023-05-23 20:56:28
 * @LastEditors: Meng Kai
 * @LastEditTime: 2023-05-30 00:32:44
*/
#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include "color_terminal/color_terminal.hpp"
namespace SlamCraft
{
    class SCModuleBase
    {
    private:
        YAML::Node config_node;
        std::string name;
        ctl::table_out *to_ptr;
    protected:
        SCModuleBase(const std::string&config_path,const std::string&prefix, const std::string & module_name = "default"){
            name = module_name;
            to_ptr = new ctl::table_out();
            if(config_path!=""){
                try{
                    config_node = YAML::LoadFile(config_path);
                }catch (YAML::Exception &e){
                    std::cout<<e.msg<<std::endl;
                }
                
                if(prefix!=""&&config_node[prefix])config_node = config_node[prefix];
            }
        }
        template<typename T>
        void readParam(const std::string &key,T&val,T default_val){
            if(config_node[key]){
                val = config_node[key].as<T>();
            }else{
                val = default_val;
            }
            to_ptr->add_item(key,VAR_NAME(val),val);
        }
        void printParamTable(){
            if(to_ptr!=nullptr){
                to_ptr->table_name = name;
                to_ptr->line_color.front_color = ctl::COLOR::YELLOW;
                to_ptr->line_color.ctl = ctl::CTRL::HL;
                to_ptr->make_table_and_out();

                delete to_ptr;
            }
        }
    };
} // namespace SlamCraft
 