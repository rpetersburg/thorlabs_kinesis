#!/usr/bin/env python
NAME = 'thorlabs_kinesis'
AUTHOR = 'Ryan Petersburg'
AUTHOR_EMAIL = 'ryan.petersburg@yale.edu'
LICENSE = 'MIT'
URL = 'https://github.com/rpetersburg/thorlabs_kinesis'
DESCRIPTION = 'Python Wrappers for Thorlabs Kinesis libraries'
PACKAGES = ['thorlabs_kinesis']
PLATFORMS = ['Windows']
MAJOR               = 0
MINOR               = 0
VERSION             = '{}.{}'.format(MAJOR, MINOR)

if __name__=='__main__':

    from setuptools import setup

    setup(
        name = NAME,
        version = VERSION,
        author = AUTHOR,
        author_email = AUTHOR_EMAIL,
        description = DESCRIPTION,
        license = LICENSE,
        url = URL,
        platforms = PLATFORMS,
        packages = PACKAGES,
    )
