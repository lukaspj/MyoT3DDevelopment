<?php
//-----------------------------------------------------------------------------
// Copyright (c) 2012 GarageGames, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//-----------------------------------------------------------------------------

///
///    Project info
///
class Project
{
   public static $TYPE_APP    =     'app';
   public static $TYPE_SHARED_APP = 'sharedapp';
   public static $TYPE_LIB    =     'lib';
   public static $TYPE_SHARED_LIB = 'shared';
   public static $TYPE_ACTIVEX =    'activex';
   public static $TYPE_SAFARI =     'safari';
   public static $TYPE_CSPROJECT =     'csproj';
   
   public $name;               // Project name
   public $guid;               // Project GUID
   public $type;               // Application or Library?
   public $dir_list;           // What directories are we checking in?
   public $outputs;            // List of outputs we want to generate.
   public $game_dir;           // Base product path
   public $defines;            // Preprocessor directives
   public $disabledWarnings;   // Additional warnings to disable
   public $includes;           // Additional include paths
   public $libs;               // Additional libraries to link against
   public $libsDebug;          // Additional Debug build libraries to link against
   public $libsIgnore;         // Ignore Specific Default Libraries
   public $lib_dirs;           // Additional library search paths
   public $lib_includes;        // libs to include (generated by modules)
   public $fileCopyPaths;     // Source and desitnation (relative to project) paths of files to copy into project
   public $additionalExePath;  // Additional section to inject into executable path
   public $dependencies;       // Projects this project depends on
   public $references;         // for managed projects, references to required assemblies
   public $moduleDefinitionFile;       // definition file to control shared library exports on windows
   public $projectFileExt;
   
   public $commandDebug = "";
   public $commandOptimized = "";
   public $commandRelease = "";

   public $argsDebug = "";
   public $argsOptimized = "";
   public $argsRelease = "";
   
   public $projSubSystem = 2; // support for Windows/Console/Assembly linker subsystem (1 - Console, 2 - Windows, 3 - Assembly)

   private static $xUID = 1; // used for unique file IDs for Xcode projects
   
   public $uniformOutputFile = 0; // debug/release builds use same filename (necessary for np plugin)

   // $additionalExePath, $lib_dirs, $libs, all appear to be unused. [pauls 11/9/2007]
    public function Project( $name, $type, $guid = '', $game_dir = 'game', $output_name = '' )
    {
        if (strlen($output_name) == 0)
           $output_name = $name;

        $this->name         = $name;
        $this->outputName   = $output_name;
        $this->guid         = $guid;
        $this->type         = $type;
        $this->game_dir     = $game_dir;
        $this->dir_list     = array();
        $this->defines      = array();
        $this->includes     = array();
        $this->libs         = array();
        $this->libsDebug    = array();
        $this->libsIgnore   = array();
        $this->lib_dirs     = array();
        $this->lib_includes = array();
        $this->fileCopyPaths = array();
        $this->outputs      = array();
        $this->dependencies = array();
        $this->disabledWarnings = array();
        $this->references = array();
    }

    public function isApp()
    {
        return $this->type == self::$TYPE_APP;
    }

    public function isSharedApp()
    {
        return $this->type == self::$TYPE_SHARED_APP;
    }

    public function isLib()
    {
        return $this->type == self::$TYPE_LIB;
    }

    public function isSharedLib()
    {
        return $this->type == self::$TYPE_SHARED_LIB;
    }

    public function isCSProject()
    {
        return $this->type == self::$TYPE_CSPROJECT;
    }

    public function isActiveX()
    {
        return $this->type == self::$TYPE_ACTIVEX;
    }

    public function isSafari()
    {
        return $this->type == self::$TYPE_SAFARI;
    }

    public function setUniformOutputFile()
    {
        return $this->uniformOutputFile = 1;
    }
    
    public function setSubSystem( $subSystem )
    {
        $this->projSubSystem = $subSystem;
    }

    public function validate()
    {
        // Sort the path list
        sort( $this->dir_list );

        // Make sure we don't have any duplicate paths
        $this->dir_list = array_unique( $this->dir_list );
    }
    
    public function addReference($refName, $version = "")
    {
        $this->references[$refName] = $version;
    }

    public function addIncludes( $includes )
    {
        $this->includes = array_merge( $includes, $this->includes );
    }

    public function validateDependencies()
    {
        $pguids = array();

        foreach( $this->dependencies as $pname )
        {
            $p = Generator::lookupProjectByName( $pname );

            if( $p )
                array_push( $pguids, $p->guid );
            else
                trigger_error( "Project dependency not found:  " .$pname, E_USER_ERROR );
        }
        // todo: change to dependencyGuids
        $this->dependencies = $pguids;
    }

    private function generateXUID()
    {
        return sprintf( "%023X", Project::$xUID++ );
    }

    private function createFileEntry( $output, $curPath, $curFile )
    {
        // See if we need to reject it based on our rules..
        if( $output->ruleReject( $curFile ) )
            return null;

        // Get the extension - is it one of our allowed values?
        if( !$output->allowedFileExt( $curFile ) )
            return null;

        // Cool - note in the list!
        $newEntry       = new stdClass();
        $newEntry->name = $curFile;
        $newEntry->path = FileUtil::collapsePath( $curPath . "/" . $curFile );
        
        if ( !FileUtil::isAbsolutePath( $newEntry->path ) )
        {
           // This could be consolidated into a single OR statement but it is easier to
           // read as two separate if's
           if ( !Generator::$absPath )
              $newEntry->path = $output->project_rel_path . $newEntry->path;
              
           if ( Generator::$absPath && !stristr($newEntry->path, Generator::$absPath) )
              $newEntry->path = $output->project_rel_path . $newEntry->path;
         }
         
        // Store a project-unique ID here for Xcode projects
        //  It will be appended by a single char in the templates.
        $newEntry->hash = Project::generateXUID();

        return $newEntry;
    }

    function generateFileList( &$projectFiles, $outputName, &$output )
    {
        $projName                   = $this->name;
        $projectFiles[ $projName ]  = array();

        foreach( $this->dir_list as $dir )
        {
            $dir = FileUtil::normalizeSlashes( $dir );
            
            // Build the path.
            if ( FileUtil::isAbsolutePath( $dir ) )
               $curPath  = $dir;
            else
               $curPath  = FileUtil::collapsePath( $output->base_dir . $dir );
            $pathWalk = &$projectFiles[ $projName ];
            
            if ( Generator::$absPath )
            {
               if ( stristr($curPath, getEngineSrcDir()) || stristr($curPath, getLibSrcDir()) )
                  $curPath = Generator::$absPath . "/". str_replace("../", "", $curPath);
            }

            // Check if its a file or a directory.
            // If its a file just add it directly and build a containng filter/folder structure,
            // for it else if a dir add all files in it.
            if( is_file( $curPath ) )
            {
                // Get the file name
                $curFile = basename( $curPath );
                $curPath = dirname( $curPath );

                //echo( "FILE: " . $curFile . " PATH: " . $curPath . "\n" );
            }

            if( is_dir( $curPath ) )
            {
                //echo( "DIR: " . $curPath . "\n" );

                // Get the array we'll be adding things to...
                $pathParts = explode( '/', FileUtil::collapsePath( $dir ) );

                foreach( $pathParts as $part )
                {
                    // Skip parts that are relative paths - only want meaningful directories.
                    if( $part == '..' )
                        continue;

                    if( !is_array( $pathWalk[ $part ] ) )
                        $pathWalk[ $part ] = array();

                    $pathWalk = &$pathWalk[ $part ];
                }

                // Open directory.
                //echo( "SCANNING: " . $curPath . "\n");

                $dirHdl = opendir( $curPath );

                if( !$dirHdl )
                {
                    echo( "Path " . $curPath . " not found, giving up.\n" );

                    return false;
                }

                // Iterate over all the files in the path if not a single file spec.
                if( !$curFile )
                {
                    while( $curFile = readdir( $dirHdl ) )
                    {
                        // Skip out if it's an uninteresting dir...
                        if( $curFile == '.' || $curFile == '..' || $curFile == '.svn' || $curFile == 'CVS' )
                            continue;

                        $newEntry = $this->createFileEntry( $output, $curPath, $curFile );

                        if( $newEntry )
                            $pathWalk[] = $newEntry;
                    }
                }
                else
                {
                    $newEntry = $this->createFileEntry( $output, $curPath, $curFile );

                    if( $newEntry )
                        $pathWalk = $newEntry;

                    $curFile = '';
                }

                // Clean up after ourselves!
                closedir( $dirHdl );
            }
        }

        FileUtil::trimFileList( $projectFiles );

        // Uncomment me to see the structure the file lister is returning.
        //print_r($projectFiles);

        return true;
    }

    private function setTemplateParams( $tpl, $output, &$projectFiles )
    {
        // Set the template delimiters
        $tpl->left_delimiter  = $output->ldelim  ? $output->ldelim  : '{';
        $tpl->right_delimiter = $output->rdelim ? $output->rdelim : '}';
        $gameProjectName = getGameProjectName();

        // Evaluate template into a file.
        $tpl->assign_by_ref( 'projSettings', $this );
        $tpl->assign_by_ref( 'projOutput',   $output );
        $tpl->assign_by_ref( 'fileArray',    $projectFiles );
        $tpl->assign_by_ref( 'projName',     $this->name );
        $tpl->assign_by_ref( 'projOutName',  $this->outputName );
        $tpl->assign_by_ref( 'gameFolder',   $this->game_dir );
        $tpl->assign_by_ref( 'GUID',         $this->guid );
        $tpl->assign_by_ref( 'projDefines',  $this->defines );
        $tpl->assign_by_ref( 'projDisabledWarnings',  $this->disabledWarnings );
        $tpl->assign_by_ref( 'projIncludes', $this->includes );
        $tpl->assign_by_ref( 'projLibs',     $this->libs );
        $tpl->assign_by_ref( 'projLibsDebug',$this->libsDebug);
        $tpl->assign_by_ref( 'projLibsIgnore',$this->libsIgnore);
        $tpl->assign_by_ref( 'projLibDirs',  $this->lib_dirs );
        $tpl->assign_by_ref( 'projDepend',   $this->dependencies );
        $tpl->assign_by_ref( 'gameProjectName', $gameProjectName );
        $tpl->assign_by_ref( 'projModuleDefinitionFile',   $this->moduleDefinitionFile );
        $tpl->assign_by_ref( 'projSubSystem', $this->projSubSystem );
        
        if (Generator::$useDLLRuntime)
        {
            // /MD and /MDd
            $tpl->assign( 'projRuntimeRelease', 2 );
            $tpl->assign( 'projRuntimeDebug', 3 );
        }
        else
        {
            // /MT and /MTd
            $tpl->assign( 'projRuntimeRelease', 0 );
            $tpl->assign( 'projRuntimeDebug', 1 );       
        }
        
        if (!$this->commandDebug && ( $this->isSharedLib() || $this->isSharedApp() ))
        {
        
            $command = "$(TargetDir)\\".$this->outputName;
            $tpl->assign( 'commandDebug' , $command."_DEBUG.exe");
            $tpl->assign( 'commandRelease' , $command.".exe");
            $tpl->assign( 'commandOptimized' , $command."_OPTIMIZEDDEBUG.exe");
        }
        else
        {
            $tpl->assign_by_ref( 'commandDebug' , $this->commandDebug);
            $tpl->assign_by_ref( 'commandRelease' , $this->commandRelease);
            $tpl->assign_by_ref( 'commandOptimized' , $this->commandOptimized);
        }

        $tpl->assign_by_ref( 'argsDebug' , $this->argsDebug);
        $tpl->assign_by_ref( 'argsRelease' , $this->argsRelease);
        $tpl->assign_by_ref( 'argsOptimized' , $this->argsOptimized);
                
        $ptypes = array();
        $projectDepends = array();
        
        foreach ($this->dependencies as $pname)
        {          
          $p = Generator::lookupProjectByName( $pname );
          $projectDepends[$pname] = $p;
          
          if ( $p )
            $ptypes[$pname] = $p->isSharedLib() || $p->isSafari();
        }
        
        $tpl->assign_by_ref( 'projTypes',   $ptypes );
        $tpl->assign_by_ref( 'projectDepends',   $projectDepends );
          
        // Assign some handy paths for the template to reference
        $tpl->assign( 'projectOffset', $output->project_rel_path );
        
        if ( Generator::$absPath )
           $tpl->assign( 'srcDir', Generator::$absPath . "/". str_replace("../", "", getAppEngineSrcDir()) );
        else
           $tpl->assign( 'srcDir', $output->project_rel_path . getAppEngineSrcDir() );
           
        if ( Generator::$absPath )
           $tpl->assign( 'libDir', Generator::$absPath . "/". str_replace("../", "", getAppLibSrcDir()) );
        else
           $tpl->assign( 'libDir', $output->project_rel_path . getAppLibSrcDir() );
        
        if ( Generator::$absPath )
           $tpl->assign( 'binDir', Generator::$absPath . "/". str_replace("../", "", getAppEngineBinDir()) );
        else
           $tpl->assign( 'binDir', $output->project_rel_path . getAppEngineBinDir() );
           
        $tpl->assign( 'uniformOutputFile', $this->uniformOutputFile);                
    }
        
    
    private function conditionDirectories( $output, &$projectFiles )
   {
      foreach ($this->includes as &$include)
      {
         if ( !FileUtil::isAbsolutePath( $include ) )
            $include = $output->project_rel_path . $include;
      }
       
      foreach ($this->lib_dirs as &$libDirs)
      {
         if ( !FileUtil::isAbsolutePath( $libDirs ) )
            $libDirs = $output->project_rel_path . $libDirs;
      }

       if ( Generator::$absPath )
       {
          foreach ($this->includes as &$include)
          {
             if ( stristr($include, getEngineSrcDir()) || stristr($include, getLibSrcDir()) )
               $include = Generator::$absPath . "/". str_replace("../", "", $include);
          }
             
          foreach ($this->lib_dirs as &$libDirs)
          {
             if ( stristr($libDirs, getEngineSrcDir()) || stristr($libDirs, getLibSrcDir()) )
                $libDirs = Generator::$absPath . "/". str_replace("../", "", $libDirs);
          }
       }
    }

    public function generate( $tpl, $platform, $base_dir )
    {
        // Alright, for each project scan and generate the file list.
        $projectFiles    = array ();
        $rootPhpBuildDir = getcwd();

        // Iterate over this project's outputs.
        foreach( $this->outputs as $outputName => $output )
        {
            $saved_includes = $this->includes;
            $saved_lib_dirs = $this->lib_dirs;
            
            //print_r( $output );

            // Supported platform?
            if( !$output->supportsPlatform( $platform ) )
            {
                //echo( "      # Skipping output: '$outputName'.\n" );

                continue;
            }

            // Get to the right working directory (first go back to root, then to relative)
            chdir( $base_dir );

            //echo( "      - Changing CWD to " . $output->output_dir . "\n" );
            // echo("        (From: " . getcwd() . ")\n");

            if( !FileUtil::prepareOutputDir( $output->output_dir ) )
                continue;

            //echo( "      - Scanning directory for output  '.$outputName.'...\n" );

            if( !$this->generateFileList( $projectFiles, $outputName, $output ) )
            {
                echo( "File list generation failed. Giving up on this project.\n" );

                continue;
            }

            // Do any special work on the include/lib directories that we need
            $this->conditionDirectories( $output, $projectFiles[ $this->name ] );
            
            $this->projectFileExt = $output->output_ext;
            
            if ( $this->isCSProject() )
              $this->projectFileExt = ".csproj"; // always csproj C# project under VS/MonoDevelop

            $outfile = $output->project_dir . $this->name . $this->projectFileExt;

            echo( "      o Writing project file " . $outfile . "\n" );

            $this->setTemplateParams( $tpl, $output, $projectFiles[ $this->name ] );
            // To put a bandaid on the tools/player output dir problem
            // CodeReview: This should be in the template. -- BJG, 3/13/2007
            // Moved into templates -- neo

            // Write file
            $outdir = dirname( $outfile );

            if( !file_exists( $outdir ) )
                mkdir_r( $outdir, 0777 );

            if( $hdl = fopen( $outfile, 'w' ) )
            {
                if ($this->isApp())
                   $template = $output->template_app;
                else if ($this->isLib())
                   $template = $output->template_lib;
                else if ($this->isSharedLib())
                   $template = $output->template_shared_lib;
                else if ($this->isSharedApp())
                   $template = $output->template_shared_app;
                else if ($this->isActiveX())
                   $template = $output->template_activex;
                else if ($this->isSafari())
                   $template = $output->template_activex; //rename template?
                else if ($this->isCSProject())
                  $template = $output->template_csproj;

                fputs( $hdl, $tpl->fetch( $template ) );

                fclose( $hdl );
            }
            else
                trigger_error( "Could not write output file: " . $output->outputFile, E_USER_ERROR );
                            
            if ($output->template_user)
            {
            
                $outfile = $output->project_dir . $this->name . $this->projectFileExt .'.'.getenv("COMPUTERNAME").'.'.getenv("USERNAME").'.user';
                
                if( !file_exists( $outfile ) )
                {
                    if( $hdl = fopen( $outfile, 'w' ) )
                    {
                        $template = $output->template_user;
                        fputs( $hdl, $tpl->fetch( $template ) );
                        fclose( $hdl );
                    }
                    else
                        trigger_error( "Could not write output file: " . $outfile, E_USER_ERROR );
                }
               
            }

            // Build the .filters file used by VS2010.
            if ( $output->template_filter )
            {
               $filterData = new FilterData();
               array_walk( $projectFiles[ $this->name ], array($filterData, 'callback'), '' );

               $tpl->assign_by_ref('Folders', $filterData->folders);
               $tpl->assign_by_ref('SrcFiles', $filterData->srcFiles);
               $tpl->assign_by_ref('IncFiles', $filterData->incFiles);
               $tpl->assign_by_ref('OtherFiles', $filterData->otherFiles);
               $tpl->register_function( 'gen_uuid', 'gen_uuid' );

               $outfile = $output->project_dir . $this->name . $this->projectFileExt . '.filters';
               if ( $hdl = fopen( $outfile, 'w' ) )
               {
                  fputs( $hdl, $tpl->fetch( $output->template_filter ) );
                  fclose( $hdl );
               }
            }

            $this->includes = $saved_includes;
            $this->lib_dirs = $saved_lib_dirs;
        }
        
        // Copy any files into the project
        foreach( $this->fileCopyPaths as $paths )
        {
            $source = $paths[0];
            $dest = $paths[1];

            // We need forward slashes for paths.
            $source = str_replace( "\\", "/", $source);
            $dest = str_replace( "\\", "/", $dest);

            // Remove trailing slashes.
            $source = rtrim($source, " /");
            $dest = rtrim($dest, " /");
            
            // Remove any beginning slash from the destination
            $dest = ltrim($dest, " /");
            
            // Build full destination path
            $fullDest = $base_dir . "/" . $dest;
      
            echo( "      o Copying file " . $source . " to " . $fullDest . "\n" );
            if(!copy($source, $fullDest))
            {
                trigger_error(
                      "\n*******************************************************************".
                      "\n".
                      "\n  Unable to copy required file for project!".
                      "\n".
                      "Source file: " . $source . "\n" .
                      "Destination file: " . $fullDest . "\n" .
                      "\n".
                      "\n*******************************************************************".
                      "\n", E_USER_ERROR );
            }
        }
    }
}


class FilterData
{
   public $folders = array();
   public $srcFiles = array();
   public $incFiles = array();
   public $otherFiles = array();

   public function callback( $value, $key, $dir )
   {
      if ( is_array( $value ) )
      {
         if ( $dir != '' )
            $dirpath = $dir . '\\' . $key;
         else
            $dirpath = $key;

         array_push( $this->folders, $dirpath );
         array_walk( $value, array($this, 'callback'), $dirpath );
         return;
      }

      $path = str_replace( '/', '\\', $value->path );

      $ext = strrchr( $path, '.' );
      if ( $ext == FALSE )
         return;

      if (  strcasecmp( $ext, '.c' ) == 0 ||
            strcasecmp( $ext, '.cpp' ) == 0 ||
            strcasecmp( $ext, '.cc' ) == 0 )
         $this->srcFiles[$path] = $dir;

      else if (   strcasecmp( $ext, '.h' ) == 0 ||
                  strcasecmp( $ext, '.hpp' ) == 0 ||
                  strcasecmp( $ext, '.inl' ) == 0 )
         $this->incFiles[$path] = $dir;

      else
         $this->otherFiles[$path] = $dir;

   }

} // class FilterData

?>
