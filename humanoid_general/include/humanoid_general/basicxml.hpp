#ifndef BASICXML_HPP
#define BASICXML_HPP
#include <libxml/parser.h>
#include <libxml/xmlmemory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <string>
#include <exception>

// Definir exceções com mensagens de maneira simples
#define simpleException(classname, message) class classname:public std::exception{virtual const char*what()const throw(){return message;}}
simpleException(WrongNode, "Algum nó xml não está correto");
simpleException(MissingProperty, "Faltando propriedade XML requerida");
simpleException(FileError, "Não conseguiu abrir arquivo XML");
simpleException(RootNodeError, "O arquivo não possui nó root válido");

/* isnode: retorna se um nó x tem o nome s
 * Parâmetros:
 * x = ponteiro para o nó cujo nome deve ser verificado
 * s = string com o nome a ser comparado
 * Retorno: verdadeiro se as strings conferem, falso caso contrário
 */
bool isnode(xmlNodePtr x, const char* s);

/* parseprop2int: retorna o inteiro equivalente a uma propriedade do nó xml
 * Parâmetros:
 * node = ponteiro para o nó cuja propriedade deve ser retornada
 * prop = nome da propriedade
 * std = valor a retornar por padrão (em caso de inexistência)
 * Retorno: o valor inteiro da propriedade
 */
int parseprop2int(xmlNodePtr node, const char* prop, int std);

/* parseprop2double: retorna o double equivalente a uma propriedade do nó xml
 * Parâmetros:
 * node = ponteiro para o nó cuja propriedade deve ser retornada
 * prop = nome da propriedade
 * std = valor a retornar por padrão (em caso de inexistência)
 * Retorno: o valor double da propriedade
 */
double parseprop2double(xmlNodePtr node, const char* prop, double std);

/* parseprop2string: retorna a string c++ equivalente a uma propriedade do nó xml
 * Parâmetros:
 * node = ponteiro para o nó cuja propriedade deve ser retornada
 * prop = nome da propriedade
 * std = valor a retornar por padrão (em caso de inexistência)
 * Retorno: o valor double da propriedade
 */
std::string parseprop2string(xmlNodePtr node, const char* prop, std::string std);

/* node2int: retorna o inteiro dentro de um nó xml
 * Parâmetros:
 * node = ponteiro para o nó que deve ser lido
 * std = valor a retornar por padrão (em caso de inexistência)
 * Retorno: o valor inteiro do nó
 */
int node2int(xmlNodePtr node, int std);

/* node2double: retorna o double dentro de um nó xml
 * Parâmetros:
 * node = ponteiro para o nó que deve ser lido
 * std = valor a retornar por padrão (em caso de inexistência)
 * Retorno: o valor do nó
 */
double node2double(xmlNodePtr node, double std);

/* setintprop: salva um inteiro como atributo xml
 * Parâmetros:
 * node = ponteiro para o nó de destino
 * prop = nome do atributo
 * value = valor do atributo a ser salvo
 * Retorno: void
 */
void setintprop(xmlNodePtr node, const char *prop, int value);

/* setdoubleprop: salva um double como atributo xml
 * Parâmetros:
 * node = ponteiro para o nó de destino
 * prop = nome do atributo
 * value = valor do atributo a ser salvo
 * Retorno: void
 */
void setdoubleprop(xmlNodePtr node, const char *prop, double value);

/* setstringprop: salva uma string como atributo xml
 * Parâmetros:
 * node = ponteiro para o nó de destino
 * prop = nome do atributo
 * value = valor do atributo a ser salvo
 * Retorno: void
 */
void setstringprop(xmlNodePtr node, const char *prop, std::string value);

/* newintnode: salva um inteiro como um elemento xml
 * Parâmetros:
 * pai = nó para adicionar esse inteiro
 * nome = nome do novo nó
 * value = valor do conteúdo do nó
 * Retorno: o nó recém-criado
 */
xmlNodePtr newintnode(xmlNodePtr pai, const char* nome, int value);

/* file2xmlnode: retorna o nó raiz de um arquivo
 * Parâmetros:
 * filename = nome do arquivo a ser aberto
 * Retorno: o nó raiz do arquivo
 * Note que precisa fechar o arquivo depois de usar
 */
xmlNodePtr file2xmlnode(const char *filename);

/* xmlnode2file: salva um nó como raiz de um arquivo e limpa-o da memória
 * Parâmetros:
 * node = ponteiro para o nó a ser salvo
 * filename = nome do arquivo a ser salvo/sobrescrito
 * Retorno: void
 */
void xmlnode2file(xmlNodePtr node, const char *filename);


std::string toString(double value); //por algum motivo desconhecido o compilador não reconhece a função to_string
double string2double(const std::string &in);

#endif /* BASICXML_HPP */
