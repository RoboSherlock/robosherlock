#ifndef KRDEFINITIONS_H
#define KRDEFINITIONS_H

#include <map>
#include <vector>
#include <string>
#include <sstream>


/* for now this is OK, but to scale a diferent strategy needs to be implemented
 * - main issue here is that classifiers are not trained on the names in knowrob so we need a mapping
 * - the query terms should be stored somewhere else to facilitate easy extenstion
*/
namespace rs_queryanswering
{

static std::vector<std::string> krNamespaces =
{
  "knowrob",
  "kitchen",
  "rs_components"
};

static std::map<std::string, std::string> krNameMapping =
{
  //superclasses
  {"Drink", "knowrob:'Drink'"},
  {"FoodOrDrinkOrIngredient", "knowrob:'FoodOrDrinkOrIngredient'"},
  {"Container", "knowrob:'Container'"},
  {"CookingUtensil", "knowrob:'CookingUtensil'"},
  {"ElectricalDevice", "knowrob:'ElectricalDevice'"},
  {"Cutlery", "kitchen:'Cutlery'"},
  {"Fork", "knowrob:'Fork'"},
  {"Knife", "knowrob:'Knife'"},
  {"Spoon", "knowrob:'Spoon'"},
  {"Cup", "knowrob:'Cup'"},
  {"Bottle", "knowrob:'Bottle'"},
  {"Plate", "knowrob:'Plate'"},
  {"Bowl", "knowrob:'Bowl'"},
  {"Milk", "knowrob:'Milk'"},
  {"KoellnMuesliKnusperHonigNuss", "kitchen:'KoellnMuesliKnusperHonigNuss'"},
  {"BreakfastCereal","knowrob:'BreakfastCereal'"},


  //OBJECTS:
  //kitchen should solve this with some conversion and naming guideline
  {"icetea", "kitchen:'PfannerPfirschIcetea'"},
  {"mondamin", "kitchen:'Mondamin'"},
  {"cereal", "kitchen:'KellogsCornFlakes'"},
  {"red_spotted_plate", "kitchen:'RedSpottedPlate'"},
  {"pancake_maker", "kitchen:'PancakeMaker'"},
  {"spatula", "kitchen:'FryingSpatula'"},
  {"pitcher", "kitchen:'YellowPitcher'"},
  {"milk", "kitchen:'VollMilch'"},
  {"cup", "kitchen:'BlueCup'"},
  {"albi_himbeer_juice", "kitchen:'AlbiHimbeerJuice'"},
  {"black_bowl", "kitchen:'BlackBowl'"},
  {"blue_cup", "kitchen:'BlueCup'"},
  {"blue_plastic_bowl", "kitchen:'BluePlasticBowl'"},
  {"blue_spotted_plate", "kitchen:'BlueSpottedPlate'"},
  {"cappuccino", "kitchen:'Cappuccino'"},
  {"coffee_el_bryg", "kitchen:'CoffeeElBryg'"},
  {"fork_blue_plastic", "kitchen:'ForkBluePlastic'"},
  {"fork_red_plastic", "kitchen:'ForkRedPlastic'"},
  {"fork_ycb", "kitchen:'ForkYcb'"},
  {"frying_pan", "kitchen:'FryingPan'"},
  {"frying_spatula", "kitchen:'FryingSpatula'"},
  {"hela_curry_ketchup", "kitchen:'HelaCurryKetchup'"},
  {"ja_milch", "kitchen:'JaMilch'"},
  {"jod_salz", "kitchen:'JodSalz'"},
  {"kelloggs_corn_flakes", "kitchen:'KelloggsCornFlakes'"},
  {"kelloggs_toppas_mini", "kitchen:'KellogsToppasMini'"},
  {"knife_blue_plastic", "kitchen:'KnifeBluePlastic'"},
  {"knife_red_plastic", "kitchen:'KnifeRedPlastic'"},
  {"knife_ycb", "kitchen:'KnifeYcb'"},
  {"knusper_schoko_keks", "kitchen:'KnusperSchokoKeks'"},
  {"large_grey_spoon", "kitchen:'LargeGreySpoon'"},
  {"linux_cup", "kitchen:'LinuxCup'"},
  {"LionCerealBox", "kitchen:'LionCerealBox'"},
  {"marken_salz", "kitchen:'MarkenSalz'"},
  {"meer_salz", "kitchen:'MeerSalz'"},
  {"mondamin", "kitchen:'Mondamin'"},
  {"nesquik_cereal", "kitchen:'NesquikCereal'"},
  {"pancake_maker", "kitchen:'PancakeMaker'"},
  {"pfanner_grune_icetea", "kitchen:'PfannerGruneIcetea'"},
  {"pfanner_pfirsich_icetea", "kitchen:'PfannerPfirsichIcetea'"},
  {"pitcher_white", "kitchen:'PitcherWhite'"},
  {"pitcher_yellow", "kitchen:'PitcherYellow'"},
  {"pot", "kitchen:'Pot'"},
  {"pringles_paprika", "kitchen:'PringlesPaptrika'"},
  {"pringles_salt", "kitchen:'PringlesSalt'"},
  {"pringles_vinegar", "kitchen:'PringlesVinegar'"},
  {"red_spotted_bowl", "kitchen:'RedSpottedBowl'"},
  {"red_spotted_cup", "kitchen:'RedSpottedCup'"},
  {"red_metal_plate_white_speckles", "kitchen:'RedMetalPlateWhiteSpeckles'"},
  {"reine_butter_milch", "kitchen:'ReineButterMilch'"},
  {"slotted_spatula", "kitchen:'SlottedSpatula'"},
  {"soja_milch", "kitchen:'SojaMilch'"},
  {"spitzen_reis", "kitchen:'SpitzenReis'"},
  {"toaster", "kitchen:'Toaster'"},
  {"tomato_al_gusto_basilikum", "kitchen:'TomatoAlGustoBasilikum'"},
  {"tomato_sauce_oro_di_parma", "kitchen:'TomatoSauceOroDiParma'"},
  {"VollMilch", "kitchen:'VollMilch'"},
  {"white_bowl", "kitchen:'WhiteBowl'"},
  {"yellow_plate", "kitchen:'YellowPlate'"},
  {"cup_eco_orange", "kitchen:'CupEcoOrange'"},
  {"blue_camping_cup", "kitchen:'BlueCampingCup'"},
  {"sigg_bottle", "kitchen:'SiggBottle'"},
  {"white_bottle", "kitchen:'WhiteBottle'"},
  {"pancake","kitchen:'Pancake'"},

  {"spoon_blue_plastic", "kitchen:'SpoonBluePlastic'"},
  {"edeka_red_bowl", "kitchen:'EdekaRedBowl'"},
  {"weide_milch_small", "kitchen:'WeideMilchSmall'"},
  {"koelln_muesli_knusper_honig_nuss", "kitchen:'KoellnMuesliKnusperHonigNuss'"},
  

  //Chemlab
  {"bottle_kimax", "kitchen:'KimaxBottle'"},
  {"bottle_acid", "kitchen:'KimaxBottle'"},
  {"bottle_base", "kitchen:'KimaxBottle'"},
  {"flask_250ml", "kitchen:'Flask'"},
  {"flask_400ml", "kitchen:'Flask'"},
  {"pipette", "kitchen:'Pipette'"},
  {"mixer_ikamag", "kitchen:'MixerIkaMag'"},


  //assembly
  {"ChassisHolder","'http://knowrob.org/kb/thorin_simulation.owl#ChassisHolder'"},
  {"AxleHolder","'http://knowrob.org/kb/thorin_simulation.owl#AxleHolder'"},
  {"AccessoryHolder","'http://knowrob.org/kb/thorin_simulation.owl#AccessoryHolder'"},
  {"CamaroBody","'http://knowrob.org/kb/thorin_simulation.owl#CamaroBody'"}
};



static std::vector<std::string> rsQueryTerms =
{
  "shape", "volume", "contains", "color", "size","class",
  "location", "logo", "text", "product", "obj-part",
  "detection", "type", "handle", "ingredient", "cad-model"
};


//make uri from namespace and class name
static std::string makeUri(const std::string &a)
{
  std::string prefix("http://knowrob.org/kb/");
  int tokenizerIdx = a.find(":");
  std::string begin = a.substr(0, tokenizerIdx);
  std::string end = a.substr(tokenizerIdx+1, a.length()-tokenizerIdx);
  //remove quoatation marks;
  end.erase(end.end()-1);
  end.erase(end.begin());

  std::stringstream stream;
  stream<<prefix<<begin<<".owl#"<<end;
  return stream.str();
}

}//end of namespace


#endif // KRDEFINITIONS_H
