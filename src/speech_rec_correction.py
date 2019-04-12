#! /usr/bin/env python
# -*- coding: utf-8 -*-
def speech_rec_correction(rec_result):
    #corrction for locations
    if 'planet' in rec_result:
        rec_result[rec_result.index('planet')] = 'toilet'
    if 'TV' in rec_result:
        rec_result[rec_result.index('TV')] = 'TV couch'
    if 'couch' in rec_result:
        rec_result[rec_result.index('couch')] = 'TV couch'
    if 'coach' in rec_result:
        rec_result[rec_result.index('coach')] = 'TV couch'
    if 'bathing' in rec_result:
        rec_result[rec_result.index('bathing')] = 'washbasin'
    if 'machine' in rec_result:
        rec_result[rec_result.index('machine')] = 'washing machine'
    if 'arm' in rec_result and 'chair' in rec_result:
        if rec_result.index('arm') + 1 == rec_result.index('chair'):
            rec_result[rec_result.index('arm')]='armfchair'
            rec_result.pop(rec_result.index('chair'))
    if 'colorado' in rec_result:
        rec_result[rec_result.index('colorado')] = 'corridor'
    if 'coffee' in rec_result:
        rec_result[rec_result.index('coffee')] = 'coffee table'
    if 'tower' in rec_result:
        rec_result[rec_result.index('tower')] = 'towel rail'
    if 'towel' in rec_result:
        rec_result[rec_result.index('towel')] = 'towel rail'
#correction for room
    if 'tuna' in rec_result:
        rec_result[rec_result.index('tuna')] = 'tuna fish'
    if 'M' in rec_result:
        rec_result[rec_result.index('M')] = 'M and M\'s'
    if 'pair' in rec_result:
        rec_result[rec_result.index('pair')] = 'pear'
    if 'pairs' in rec_result:
        rec_result[rec_result.index('pairs')] = 'pear'
    if 'pitch' in rec_result:
        rec_result[rec_result.index('pitch')] = 'peach'
    if 'picture' in rec_result:
        rec_result[rec_result.index('picture')] = 'peach'
    if 'piece' in rec_result:
        rec_result[rec_result.index('piece')] = 'peach'
    return rec_result
#correction for navi
    if 'photo' in rec_result:
        rec_result[rec_result.index('photo')] = 'follow'

        

        

        

