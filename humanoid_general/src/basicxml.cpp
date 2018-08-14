#include <libxml/parser.h>
#include <libxml/xmlmemory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include "humanoid_general/basicxml.hpp"

/* isnode: retorna se um nó x tem o nome s
 * Parâmetros:
 * x = ponteiro para o nó cujo nome deve ser verificado
 * s = string com o nome a ser comparado
 * Retorno: verdadeiro se as strings conferem, falso caso contrário
 */
bool isnode(xmlNodePtr x, const char* s)
{
	return 0==xmlStrcmp((x)->name, (xmlChar*)(s));
}

/* parseprop2int: retorna o inteiro equivalente a uma propriedade do nó xml
 * Parâmetros:
 * node = ponteiro para o nó cuja propriedade deve ser retornada
 * prop = nome da propriedade
 * std = valor a retornar por padrão (em caso de inexistência)
 * Retorno: o valor inteiro da propriedade
 */
int parseprop2int(xmlNodePtr node, const char* prop, int std)
{
	xmlChar *tmp;
	int ret;
	tmp = xmlGetProp(node, (xmlChar*)prop);
	if(tmp != NULL) {
#ifdef DEBUG
		printf("Propriedade %s = %s\n", prop, (char*)tmp);
#endif
		ret = atoi((char*)tmp);
		xmlFree(tmp);
	}
	else ret = std;
	return ret;
}

/* parseprop2double: retorna o double equivalente a uma propriedade do nó xml
 * Parâmetros:
 * node = ponteiro para o nó cuja propriedade deve ser retornada
 * prop = nome da propriedade
 * std = valor a retornar por padrão (em caso de inexistência)
 * Retorno: o valor double da propriedade
 */
double parseprop2double(xmlNodePtr node, const char* prop, double std)
{
	xmlChar *tmp;
	double ret;
	tmp = xmlGetProp(node, (xmlChar*)prop);
	if(tmp != NULL) {
#ifdef DEBUG
		printf("Propriedade %s = %s\n", prop, (char*)tmp);
#endif
		ret = atof((char*)tmp);
		xmlFree(tmp);
	}
	else ret = std;
	return ret;
}

/* parseprop2string: retorna a string c++ equivalente a uma propriedade do nó xml
 * Parâmetros:
 * node = ponteiro para o nó cuja propriedade deve ser retornada
 * prop = nome da propriedade
 * std = valor a retornar por padrão (em caso de inexistência)
 * Retorno: o valor double da propriedade
 */
std::string parseprop2string(xmlNodePtr node, const char* prop, std::string std)
{
	xmlChar *tmp;
	std::string ret;
	tmp = xmlGetProp(node, (xmlChar*)prop);
	if(tmp != NULL) {
		int i=0;
#ifdef DEBUG
		printf("Propriedade %s = %s\n", prop, (char*)tmp);
#endif
		for(i=0; tmp[i]!='\0'; i++)
			ret.push_back(tmp[i]);
		xmlFree(tmp);
	}
	else ret = std;
	return ret;
}

/* node2int: retorna o inteiro dentro de um nó xml
 * Parâmetros:
 * node = ponteiro para o nó que deve ser lido
 * std = valor a retornar por padrão (em caso de inexistência)
 * Retorno: o valor inteiro do nó
 */
int node2int(xmlNodePtr node, int std)
{
	int ret;
	xmlNodePtr n = node->children;
	if(n == NULL) return std;
	ret = atoi((char*) n->content);
	return ret;
}

/* node2double: retorna o double dentro de um nó xml
 * Parâmetros:
 * node = ponteiro para o nó que deve ser lido
 * std = valor a retornar por padrão (em caso de inexistência)
 * Retorno: o valor do nó
 */
double node2double(xmlNodePtr node, double std)
{
	double ret;
	xmlNodePtr n = node->children;
	if(n == NULL) return std;
	ret = atof((char*) n->content);
	return ret;
}

/* setintprop: salva um inteiro como atributo xml
 * Parâmetros:
 * node = ponteiro para o nó de destino
 * prop = nome do atributo
 * value = valor do atributo a ser salvo
 * Retorno: void
 */
void setintprop(xmlNodePtr node, const char *prop, int value)
{
	char tmp[32];
	snprintf(tmp, sizeof(tmp), "%d", value);
	xmlNewProp(node, (xmlChar*)prop, (xmlChar*)tmp);
}

/* setdoubleprop: salva um double como atributo xml
 * Parâmetros:
 * node = ponteiro para o nó de destino
 * prop = nome do atributo
 * value = valor do atributo a ser salvo
 * Retorno: void
 */
void setdoubleprop(xmlNodePtr node, const char *prop, double value)
{
	char tmp[128];
	int i=0;
	double dez = value;
	int aprox;
	while(i<6) {
		aprox = dez;
		if(dez == aprox) break;
		dez *= 10;
		i++;
	}
	snprintf(tmp, sizeof(tmp), "%.*lf", i, value);
	xmlNewProp(node, (xmlChar*)prop, (xmlChar*)tmp);
}

/* setstringprop: salva uma string como atributo xml
 * Parâmetros:
 * node = ponteiro para o nó de destino
 * prop = nome do atributo
 * value = valor do atributo a ser salvo
 * Retorno: void
 */
void setstringprop(xmlNodePtr node, const char *prop, std::string value)
{
	char *tmp;
	unsigned int i;
	tmp = (char*)malloc((value.size()+1)*sizeof(char));
	for(i=0; i<=value.size(); i++) {
		tmp[i] = value[i];
	}
	xmlNewProp(node, (xmlChar*)prop, (xmlChar*)tmp);
	free(tmp);
}

/* newintnode: salva um inteiro como um elemento xml
 * Parâmetros:
 * pai = nó para adicionar esse inteiro
 * nome = nome do novo nó
 * value = valor do conteúdo do nó
 * Retorno: o nó recém-criado
 */
xmlNodePtr newintnode(xmlNodePtr pai, const char* nome, int value)
{
	char tmp[32];
	snprintf(tmp, sizeof(tmp), "%d", value);
	return xmlNewTextChild(pai, NULL, (xmlChar*) nome, (xmlChar*) tmp);
}

/* file2xmlnode: retorna o nó raiz de um arquivo
 * Parâmetros:
 * filename = nome do arquivo a ser aberto
 * Retorno: o nó raiz do arquivo
 * Note que precisa fechar o arquivo depois de usar
 */
xmlNodePtr file2xmlnode(const char *filename)
{
	xmlDocPtr doc;
	xmlNodePtr root;

	doc = xmlParseFile(filename);
	if(doc==NULL) throw FileError();
	root = xmlDocGetRootElement(doc);
	if(doc==NULL) throw RootNodeError();
	return root;
}

/* xmlnode2file: salva um nó como raiz de um arquivo e limpa-o da memória
 * Parâmetros:
 * node = ponteiro para o nó a ser salvo
 * filename = nome do arquivo a ser salvo/sobrescrito
 * Retorno: void
 */
void xmlnode2file(xmlNodePtr node, const char *filename)
{
	xmlDocPtr doc;
	doc = xmlNewDoc((xmlChar*)"1.0");
	xmlDocSetRootElement(doc, node);
	xmlSaveFormatFileEnc(filename, doc, "utf-8", 1);
	xmlFreeDoc(doc);
}


std::string toString(double value)
{
    std::ostringstream stm ;
    stm << value ;
    return stm.str() ;
}


double string2double(const std::string &in)
{
    std::string str = in;
    std::size_t dotPos   = str.find('.');
    std::size_t len      = str.length();
    double num = 0;
    if(dotPos != std::string::npos)
    {

        str.erase(dotPos,1);
        num = (double)atof(str.c_str())/pow(10.,double(len - dotPos-1));
    }
    else
    {
        num = (double)atof(str.c_str());
    }


    return num;
}
